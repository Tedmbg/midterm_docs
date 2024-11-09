const express = require('express');
const bodyParser = require('body-parser');
const { Pool } = require('pg');

const app = express();
const port = 3000; // Change as needed

// Middleware to parse JSON requests
app.use(bodyParser.json());

// PostgreSQL connection pool
const pool = new Pool({
    user: 'avnadmin',
    host: 'pg-3467ff9e-tedmbg9-558f.k.aivencloud.com',
    database: 'defaultdb',
    password: 'AVNS_q7FCjCAaRJSwoGS9vqh',
    port: 22282, // Default for PostgreSQL is 5432
});

// Test GET endpoint
app.get('/api/test', (req, res) => {
    res.json({ message: 'API is working!', status: 'success' });
});

// Route to insert sensor data
app.post('/add/sensor_data', async (req, res) => {
    const {
        dataid,
        timestamp,
        soilmoisture,
        temperature,
        humidity,
        valvestatus,
        waterflow,
    } = req.body;

    if (!dataid || !timestamp || !soilmoisture || !temperature || !humidity || !valvestatus || !waterflow) {
        return res.status(400).json({ error: 'Missing required fields' });
    }

    try {
        const query = `
            INSERT INTO SensorData (dataid, timestamp, soilmoisture, temperature, humidity, valvestatus, waterflow)
            VALUES ($1, $2, $3, $4, $5, $6, $7)
        `;
        await pool.query(query, [
            dataid,
            timestamp,
            soilmoisture,
            temperature,
            humidity,
            valvestatus,
            waterflow,
        ]);
        res.status(200).json({ status: 'success' });
    } catch (err) {
        console.error(err.message);
        res.status(500).json({ error: 'Database insertion failed' });
    }
});

// Start the server
app.listen(port, () => {
    console.log(`Serveris up and running :)`);
});
