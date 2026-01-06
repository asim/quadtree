package quadtree

import (
	"encoding/json"
	"os"
	"sync"
	"time"
)

// Store defines the interface for persisting quadtree points
type Store interface {
	// Save stores a point with the given ID
	Save(id string, point *Point) error
	// Load retrieves a point by ID
	Load(id string) (*Point, error)
	// Delete removes a point by ID
	Delete(id string) error
	// List returns all points in the store
	List() (map[string]*Point, error)
	// Close closes the store and performs cleanup
	Close() error
}

// StoredPoint represents a point that can be serialized
type StoredPoint struct {
	ID   string      `json:"id"`
	X    float64     `json:"x"`
	Y    float64     `json:"y"`
	Data interface{} `json:"data"`
}

// MemoryStore is an in-memory implementation of Store
type MemoryStore struct {
	mu     sync.RWMutex
	points map[string]*Point
}

// NewMemoryStore creates a new in-memory store
func NewMemoryStore() *MemoryStore {
	return &MemoryStore{
		points: make(map[string]*Point),
	}
}

// Save stores a point in memory
func (s *MemoryStore) Save(id string, point *Point) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.points[id] = point
	return nil
}

// Load retrieves a point from memory
func (s *MemoryStore) Load(id string) (*Point, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	point, exists := s.points[id]
	if !exists {
		return nil, nil
	}
	return point, nil
}

// Delete removes a point from memory
func (s *MemoryStore) Delete(id string) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	delete(s.points, id)
	return nil
}

// List returns all points from memory
func (s *MemoryStore) List() (map[string]*Point, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	// Return a copy to avoid concurrent modification
	points := make(map[string]*Point, len(s.points))
	for id, point := range s.points {
		points[id] = point
	}
	return points, nil
}

// Close is a no-op for memory store
func (s *MemoryStore) Close() error {
	return nil
}

// FileStore persists points to a JSON file with batched async writes
type FileStore struct {
	mu        sync.RWMutex
	filename  string
	points    map[string]*StoredPoint
	dirty     bool        // Has unsaved changes
	saveTimer *time.Timer // Pending save timer
	closeCh   chan struct{}
	closed    bool
}

const (
	saveDelay    = 5 * time.Second  // Delay before persisting changes
	maxSaveDelay = 30 * time.Second // Max time between saves if continuously dirty
)

// NewFileStore creates a new file-based store with batched writes
func NewFileStore(filename string) (*FileStore, error) {
	store := &FileStore{
		filename: filename,
		points:   make(map[string]*StoredPoint),
		closeCh:  make(chan struct{}),
	}

	// Try to load existing data
	if err := store.load(); err != nil {
		// If file doesn't exist, that's ok
		if !os.IsNotExist(err) {
			return nil, err
		}
	}

	// Start background saver
	go store.backgroundSaver()

	return store, nil
}

// backgroundSaver periodically saves dirty data
func (s *FileStore) backgroundSaver() {
	ticker := time.NewTicker(maxSaveDelay)
	defer ticker.Stop()

	for {
		select {
		case <-s.closeCh:
			return
		case <-ticker.C:
			s.flushIfDirty()
		}
	}
}

// flushIfDirty saves to disk if there are pending changes
func (s *FileStore) flushIfDirty() {
	s.mu.Lock()
	if !s.dirty {
		s.mu.Unlock()
		return
	}
	// Make a copy of points while holding lock
	pointsCopy := make(map[string]*StoredPoint, len(s.points))
	for k, v := range s.points {
		pointsCopy[k] = v
	}
	s.dirty = false
	s.mu.Unlock()

	// Persist without holding lock
	if err := persistPoints(s.filename, pointsCopy); err != nil {
		// Mark dirty again so we retry
		s.mu.Lock()
		s.dirty = true
		s.mu.Unlock()
	}
}

// Save stores a point (non-blocking, batched to disk)
func (s *FileStore) Save(id string, point *Point) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	x, y := point.Coordinates()
	s.points[id] = &StoredPoint{
		ID:   id,
		X:    x,
		Y:    y,
		Data: point.Data(),
	}
	s.dirty = true

	// Schedule a save after delay (debounced)
	if s.saveTimer != nil {
		s.saveTimer.Stop()
	}
	s.saveTimer = time.AfterFunc(saveDelay, func() {
		s.flushIfDirty()
	})

	return nil
}

// Load retrieves a point from the file store
func (s *FileStore) Load(id string) (*Point, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	stored, exists := s.points[id]
	if !exists {
		return nil, nil
	}

	return NewPoint(stored.X, stored.Y, stored.Data), nil
}

// Delete removes a point from the file store
func (s *FileStore) Delete(id string) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	delete(s.points, id)
	s.dirty = true
	return nil
}

// List returns all points from the file store
func (s *FileStore) List() (map[string]*Point, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()

	points := make(map[string]*Point, len(s.points))
	for id, stored := range s.points {
		points[id] = NewPoint(stored.X, stored.Y, stored.Data)
	}
	return points, nil
}

// Close saves any pending changes and stops background saver
func (s *FileStore) Close() error {
	s.mu.Lock()
	if s.closed {
		s.mu.Unlock()
		return nil
	}
	s.closed = true
	if s.saveTimer != nil {
		s.saveTimer.Stop()
	}
	close(s.closeCh)

	// Final save
	pointsCopy := make(map[string]*StoredPoint, len(s.points))
	for k, v := range s.points {
		pointsCopy[k] = v
	}
	s.mu.Unlock()

	return persistPoints(s.filename, pointsCopy)
}

// load reads the points from the file
func (s *FileStore) load() error {
	data, err := os.ReadFile(s.filename)
	if err != nil {
		return err
	}

	var points map[string]*StoredPoint
	if err := json.Unmarshal(data, &points); err != nil {
		return err
	}

	s.points = points
	return nil
}

// persistPoints writes points to file (does not hold any locks)
func persistPoints(filename string, points map[string]*StoredPoint) error {
	data, err := json.MarshalIndent(points, "", "  ")
	if err != nil {
		return err
	}

	// Write to temp file first, then rename (atomic)
	tmpFile := filename + ".tmp"
	if err := os.WriteFile(tmpFile, data, 0644); err != nil {
		return err
	}
	return os.Rename(tmpFile, filename)
}
