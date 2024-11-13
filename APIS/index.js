const express = require('express');
const bodyParser = require('body-parser');
const { Pool } = require('pg');

// Load environment variables from .env file
require('dotenv').config();

const app = express();

// Middleware to parse JSON requests
app.use(bodyParser.json());

// PostgreSQL connection pool
//here is the db
const pool = new Pool({
    user: process.env.DB_USER,
    host: process.env.DB_HOST,
    database: process.env.DB_NAME,
    password: process.env.DB_PASSWORD,
    port: process.env.DB_PORT || 5432, // Default for PostgreSQL is 5432
    ssl: {
        rejectUnauthorized: false, // Adjust this based on your database SSL settings
    },
});

// Test GET endpoint
app.get('/test', (req, res) => {
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

    if (
        !dataid ||
        !timestamp ||
        !soilmoisture ||
        !temperature ||
        !humidity ||
        !valvestatus ||
        !waterflow
    ) {
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
const port = process.env.PORT || 3000; // Use the PORT environment variable or default to 3000
app.listen(port, () => {
    console.log(`Server is up and running on port ${port}`);
});

// Optional: Export the app for testing purposes
module.exports = app;
