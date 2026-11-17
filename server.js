/*
 * Road Surface Monitor - backend
 * Express + SQLite. Receives events from the ESP32, serves them to the dashboard.
 *
 * Run:   npm install   then   API_KEY=your-secret node server.js
 * (Windows PowerShell:  $env:API_KEY="your-secret"; node server.js)
 */
const express = require('express');
const Database = require('better-sqlite3');
const path = require('path');

const PORT = process.env.PORT || 3000;
const API_KEY = process.env.API_KEY || 'change-this-secret-key';          // must match ESP32 API_KEY
const DB_PATH = process.env.DB_PATH || path.join(__dirname, 'road_events.db');

// ---------- Database ----------
const db = new Database(DB_PATH);
db.pragma('journal_mode = WAL');
db.exec(`
  CREATE TABLE IF NOT EXISTS road_events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    device_id   TEXT    NOT NULL,
    event_type  TEXT    NOT NULL CHECK (event_type IN ('pothole','bump','rough_road')),
    severity    TEXT,                       -- low / medium / high (impact strength, NOT depth)
    latitude    REAL    NOT NULL,
    longitude   REAL    NOT NULL,
    gps_fix     INTEGER NOT NULL DEFAULT 1, -- 0 = simulated/test position
    speed_kmph  REAL,
    satellites  INTEGER,
    hdop        REAL,
    max_g       REAL,                       -- highest vertical accel change in window
    min_g       REAL,                       -- lowest vertical accel change in window
    p2p_g       REAL,                       -- peak-to-peak
    gyro_dps    REAL,                       -- peak pitch/roll rate
    device_time TEXT,                       -- UTC time from GPS (ISO 8601), may be NULL
    received_at TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now'))
  );
  CREATE INDEX IF NOT EXISTS idx_events_type ON road_events(event_type);
  CREATE INDEX IF NOT EXISTS idx_events_time ON road_events(received_at);
`);

const insertStmt = db.prepare(`
  INSERT INTO road_events
    (device_id, event_type, severity, latitude, longitude, gps_fix, speed_kmph,
     satellites, hdop, max_g, min_g, p2p_g, gyro_dps, device_time)
  VALUES
    (@device_id, @event_type, @severity, @latitude, @longitude, @gps_fix, @speed_kmph,
     @satellites, @hdop, @max_g, @min_g, @p2p_g, @gyro_dps, @device_time)
`);

// ---------- Helpers ----------
const VALID_TYPES = ['pothole', 'bump', 'rough_road'];
const num = (v) => (typeof v === 'number' && Number.isFinite(v) ? v : null);

function requireKey(req, res, next) {
  if (req.get('x-api-key') !== API_KEY) return res.status(401).json({ error: 'invalid API key' });
  next();
}

// ---------- App ----------
const app = express();
app.use(express.json({ limit: '10kb' }));

app.get('/', (req, res) => res.sendFile(path.join(__dirname, 'dashboard.html')));
app.get('/api/health', (req, res) => res.json({ ok: true, time: new Date().toISOString() }));

// ESP32 -> server
app.post('/api/events', requireKey, (req, res) => {
  const b = req.body || {};
  const lat = num(b.latitude);
  const lng = num(b.longitude);

  if (!VALID_TYPES.includes(b.event_type))
    return res.status(400).json({ error: `event_type must be one of ${VALID_TYPES.join(', ')}` });
  if (lat === null || lng === null || Math.abs(lat) > 90 || Math.abs(lng) > 180)
    return res.status(400).json({ error: 'latitude/longitude missing or out of range' });

  let deviceTime = null;
  if (typeof b.device_time === 'string' && b.device_time && !Number.isNaN(Date.parse(b.device_time)))
    deviceTime = b.device_time.slice(0, 32);

  const row = {
    device_id: String(b.device_id || 'unknown').slice(0, 64),
    event_type: b.event_type,
    severity: b.severity ? String(b.severity).slice(0, 16) : null,
    latitude: lat,
    longitude: lng,
    gps_fix: b.gps_fix === false ? 0 : 1,
    speed_kmph: num(b.speed_kmph),
    satellites: Number.isInteger(b.satellites) ? b.satellites : null,
    hdop: num(b.hdop),
    max_g: num(b.max_g),
    min_g: num(b.min_g),
    p2p_g: num(b.p2p_g),
    gyro_dps: num(b.gyro_dps),
    device_time: deviceTime,
  };
  const info = insertStmt.run(row);
  console.log(`[event] #${info.lastInsertRowid} ${row.event_type} @ ${lat},${lng}`);
  res.status(201).json({ id: Number(info.lastInsertRowid) });
});

// dashboard <- server
// Query: ?types=pothole,bump  &since_hours=24  &limit=1000
app.get('/api/events', (req, res) => {
  let sql = `SELECT *, COALESCE(device_time, received_at) AS event_time FROM road_events WHERE 1=1`;
  const params = [];

  if (req.query.types) {
    const types = String(req.query.types).split(',').filter((t) => VALID_TYPES.includes(t));
    if (types.length === 0) return res.json({ count: 0, events: [] });
    sql += ` AND event_type IN (${types.map(() => '?').join(',')})`;
    params.push(...types);
  }
  const hours = Number(req.query.since_hours);
  if (Number.isFinite(hours) && hours > 0) {
    sql += ` AND received_at >= ?`;
    params.push(new Date(Date.now() - hours * 3600 * 1000).toISOString());
  }
  const limit = Math.min(Math.max(parseInt(req.query.limit, 10) || 1000, 1), 5000);
  sql += ` ORDER BY id DESC LIMIT ?`;
  params.push(limit);

  const events = db.prepare(sql).all(...params);
  res.json({ count: events.length, events });
});

app.get('/api/stats', (req, res) => {
  const rows = db.prepare(`SELECT event_type, COUNT(*) AS n FROM road_events GROUP BY event_type`).all();
  const out = { total: 0, pothole: 0, bump: 0, rough_road: 0 };
  rows.forEach((r) => { out[r.event_type] = r.n; out.total += r.n; });
  res.json(out);
});

// Clean-up helpers (need the API key) - handy while testing
app.delete('/api/events/:id', requireKey, (req, res) => {
  const info = db.prepare('DELETE FROM road_events WHERE id = ?').run(req.params.id);
  res.json({ deleted: info.changes });
});
app.delete('/api/events', requireKey, (req, res) => {
  const info = db.prepare('DELETE FROM road_events').run();
  res.json({ deleted: info.changes });
});

app.listen(PORT, '0.0.0.0', () => {
  console.log(`Road monitor server running on http://localhost:${PORT}`);
  if (API_KEY === 'change-this-secret-key') console.warn('WARNING: using the default API key - set API_KEY');
});
