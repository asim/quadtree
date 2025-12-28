# QuadTree HTTP Server

A web-based HTTP server with REST API and interactive UI for managing and visualizing points in a QuadTree.

## Features

- **REST API** for managing points (GET, POST, PUT, DELETE)
- **Interactive Web UI** with grid visualization
- **Real-time point management** - add, search, and delete points
- **Visual navigation** - mouse drag and arrow keys
- **Zoom support** - mouse wheel to zoom in/out
- **Regional search** - find points within a specific area

## Running the Server

```bash
cd cmd/server
go run main.go
```

The server will start on `http://localhost:8080`

## API Endpoints

### Add a Point
```bash
POST /api/points
Content-Type: application/json

{
  "x": 10.5,
  "y": 20.3,
  "data": "My Point"
}
```

Response:
```json
{
  "id": "1",
  "x": 10.5,
  "y": 20.3,
  "data": "My Point"
}
```

### Get All Points
```bash
GET /api/points
```

Response:
```json
[
  {
    "id": "1",
    "x": 10.5,
    "y": 20.3,
    "data": "My Point"
  }
]
```

### Get a Specific Point
```bash
GET /api/points/{id}
```

### Update a Point
```bash
PUT /api/points/{id}
Content-Type: application/json

{
  "x": 15.0,
  "y": 25.0,
  "data": "Updated Point"
}
```

### Delete a Point
```bash
DELETE /api/points/{id}
```

### Search for Points
```bash
POST /api/search
Content-Type: application/json

{
  "center": [0.0, 0.0],
  "radius": 10.0
}
```

Returns all points within the bounding box defined by center ± radius.

## Web UI

Visit `http://localhost:8080` in your browser to access the interactive UI.

### UI Features

- **Add Points**: Enter X, Y coordinates and an optional label
- **View Points**: All points are displayed on the grid with labels
- **Navigate**: 
  - Use mouse to drag the view
  - Use arrow keys (↑ ↓ ← →) to move
  - Use mouse wheel to zoom in/out
- **Search**: Set search radius and click "Search Region" to find points near current location
- **Delete**: Click the delete button next to any point to remove it
- **Reload**: Click "Reload All" to refresh and show all points

### Current Location Display

The sidebar shows:
- Current X, Y coordinates of the view center
- Current zoom level
- List of visible points with their coordinates

## Testing

Run the tests:

```bash
cd cmd/server
go test -v
```

## Examples

### Add some cities
```bash
curl -X POST http://localhost:8080/api/points \
  -H "Content-Type: application/json" \
  -d '{"x": 52.5200, "y": 13.4050, "data": "Berlin"}'

curl -X POST http://localhost:8080/api/points \
  -H "Content-Type: application/json" \
  -d '{"x": 48.8566, "y": 2.3522, "data": "Paris"}'

curl -X POST http://localhost:8080/api/points \
  -H "Content-Type: application/json" \
  -d '{"x": 51.5074, "y": -0.1278, "data": "London"}'
```

### Search for cities near a location
```bash
curl -X POST http://localhost:8080/api/search \
  -H "Content-Type: application/json" \
  -d '{"center": [50.0, 10.0], "radius": 5.0}'
```

### Delete a point
```bash
curl -X DELETE http://localhost:8080/api/points/1
```
