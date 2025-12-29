// Package ui provides a reusable web UI for quadtree visualization
package ui

import (
	"embed"
	"html/template"
	"io"
	"io/fs"
	"net/http"
)

//go:embed *.html *.js *.css
var content embed.FS

// Config contains configuration for the quadtree UI
type Config struct {
	Title           string // Page title
	APIBase         string // Base URL for API calls (e.g., "/api")
	PointsEndpoint  string // Endpoint for points CRUD (e.g., "/points")
	SearchEndpoint  string // Endpoint for search (e.g., "/search")
	ShowAddPoint    bool   // Whether to show add point controls
	ShowSearch      bool   // Whether to show search controls
	ShowDelete      bool   // Whether to show delete buttons
	ReadOnly        bool   // If true, hide all mutation controls
	PointLabel      string // Label for points (e.g., "Point", "Agent")
	AutoRefresh     bool   // Whether to auto-refresh the view
	RefreshInterval int    // Refresh interval in milliseconds (default 2000)
	ExtraCSS        string // Additional CSS to inject
	ExtraJS         string // Additional JS to inject
}

// DefaultConfig returns a sensible default configuration
func DefaultConfig() Config {
	return Config{
		Title:          "QuadTree Viewer",
		APIBase:        "/api",
		PointsEndpoint: "/points",
		SearchEndpoint: "/search",
		ShowAddPoint:   true,
		ShowSearch:     true,
		ShowDelete:     true,
		ReadOnly:       false,
		PointLabel:     "Point",
	}
}

// NetworkConfig returns configuration suitable for network visualization
func NetworkConfig() Config {
	return Config{
		Title:           "Network View",
		APIBase:         "",
		PointsEndpoint:  "/agents",
		SearchEndpoint:  "/nearby",
		ShowAddPoint:    false,
		ShowSearch:      true,
		ShowDelete:      false,
		ReadOnly:        true,
		PointLabel:      "Agent",
		AutoRefresh:     true,
		RefreshInterval: 2000,
	}
}

var tmpl *template.Template

func init() {
	var err error
	tmpl, err = template.ParseFS(content, "quadtree.html")
	if err != nil {
		panic(err)
	}
}

// Render writes the UI HTML to the given writer with the specified config
func Render(w io.Writer, cfg Config) error {
	return tmpl.Execute(w, cfg)
}

// Handler returns an http.Handler that serves the UI with the given config
func Handler(cfg Config) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		if err := Render(w, cfg); err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
		}
	})
}

// FS returns the embedded filesystem for static assets
func FS() fs.FS {
	return content
}
