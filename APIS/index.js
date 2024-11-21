const express = require("express");
const bodyParser = require("body-parser");
const { Pool } = require("pg");

// Load environment variables from .env file
require("dotenv").config();

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

//link to apis midtermdocs-production.up.railway.app

// Test GET endpoint
app.get("/test", (req, res) => {
  res.json({ message: "API is working!", status: "success" });
});

// authenticate national id from esp32
app.post("/esp/auth", async (req, res) => {
  const { nationalid } = req.body;

  // Validate request
  if (!nationalid) {
    return res.status(400).json({ error: "National ID is required" });
  }

  try {
    // Query the database for the user
    const result = await pool.query(
      "SELECT  name, nationalid, cropsplanted, dateplanted FROM users WHERE nationalid = $1",
      [nationalid]
    );

    // Check if user exists
    if (result.rows.length === 0) {
      return res.status(404).json({ error: "User not found" });
    }

    // Retrieve user data
    const user = result.rows[0];

    // Return user data as JSON
    return res.status(200).json({
      status: "success",
      message: "User authenticated successfully",
      user: {
        id: user.id,
        name: user.name,
        nationalid: user.nationalid,
        cropsplanted: user.cropsplanted || "No Crops",
        dateplanted: user.dateplanted || "No date",
      },
    });
  } catch (error) {
    console.error("Error authenticating user:", error.message);
    return res.status(500).json({ error: "Internal server error" });
  }
});

// Sign-In Route
app.post("/auth/signin", async (req, res) => {
  const { nationalid, logincredentials } = req.body;

  if (!nationalid || !logincredentials) {
    return res
      .status(400)
      .json({ error: "National ID and Password are required" });
  }

  try {
    // Check if user exists with the provided national ID
    const result = await pool.query(
      "SELECT * FROM users WHERE nationalid = $1",
      [nationalid]
    );

    if (result.rows.length === 0) {
      return res.status(401).json({ error: "Invalid National ID or Password" });
    }

    const user = result.rows[0];

    // Verify password using plain text comparison
    const isPasswordValid = logincredentials === user.logincredentials;

    if (!isPasswordValid) {
      return res.status(401).json({ error: "Invalid National ID or Password" });
    }

    // Calculate plant age
    const currentDate = new Date();
    const datePlanted = new Date(user.dateplanted);
    const plantAge = Math.floor(
      (currentDate - datePlanted) / (1000 * 60 * 60 * 24)
    ); // Age in days

    // Success - Send user details
    return res.status(200).json({
      status: "success",
      message: "Sign-in successful",
      user: {
        name: user.name,
        nationalid: user.nationalid,
        cropsplanted: user.cropsplanted,
        dateplanted: user.dateplanted,
        plantAge: plantAge,
        farmlocation: user.farmlocation,
      },
    });
  } catch (err) {
    console.error(err.message);
    res.status(500).json({ error: "Server error" });
  }
});

//Route add users
app.post("/add/user", async (req, res) => {
  const {
    name,
    contactinfo,
    logincredentials,
    farmlocation,
    soiltype,
    cropsplanted,
    nationalid,
    userid,
    age,
    dateplanted,
  } = req.body;

  // Validate required fields
  if (
    name === undefined ||
    contactinfo === undefined ||
    logincredentials === undefined ||
    farmlocation === undefined ||
    soiltype === undefined ||
    cropsplanted === undefined ||
    nationalid === undefined ||
    userid === undefined ||
    age === undefined ||
    dateplanted === undefined
  ) {
    return res.status(400).json({ error: "Missing required fields" });
  }

  // // Hash the password before saving
  // const hashedPassword = await bcrypt.hash(logincredentials, 10);

  try {
    const query = `
            INSERT INTO users (name, contactinfo, logincredentials, farmlocation, soiltype, cropsplanted, nationalid, userid, age, dateplanted)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
        `;
    await pool.query(query, [
      name,
      contactinfo,
      logincredential,
      farmlocation,
      soiltype,
      cropsplanted,
      nationalid,
      userid,
      age,
      dateplanted,
    ]);
    res
      .status(200)
      .json({ status: "success", message: "User added successfully" });
  } catch (err) {
    console.error(err.message);
    res.status(500).json({ error: "Database insertion failed" });
  }
});

// Route to insert sensor data
app.post("/add/sensor_data", async (req, res) => {
  const {
    dataid,
    timestamp,
    soilmoisture,
    temperature,
    humidity,
    valvestatus,
    waterflow,
    distance,
  } = req.body;

  if (
    dataid === undefined ||
    timestamp === undefined ||
    soilmoisture === undefined ||
    temperature === undefined ||
    humidity === undefined ||
    valvestatus === undefined ||
    waterflow === undefined ||
    distance === undefined
  ) {
    return res.status(400).json({ error: "Missing required fields" });
  }

  try {
    const query = `
            INSERT INTO sensordata (dataid, timestamp, soilmoisture, temperature, humidity, valvestatus, waterflow,distance)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
        `;
    await pool.query(query, [
      dataid,
      timestamp,
      soilmoisture,
      temperature,
      humidity,
      valvestatus,
      waterflow,
      distance,
    ]);
    res.status(200).json({ status: "success" });
  } catch (err) {
    console.error(err.message);
    res.status(500).json({ error: "Database insertion failed" });
  }
});

// Route to insert weather data
app.post("/add/weather_data", async (req, res) => {
  const {
    sunshineduration,
    timestamp,
    temperature,
    humidity,
    rainfall,
    windspeed,
    cloudcover,
    forecast,
    weatherid,
  } = req.body;

  // Validate required fields
  if (
    sunshineduration === undefined ||
    timestamp === undefined ||
    temperature === undefined ||
    humidity === undefined ||
    rainfall === undefined ||
    windspeed === undefined ||
    cloudcover === undefined ||
    forecast === undefined ||
    weatherid === undefined
  ) {
    return res.status(400).json({ error: "Missing required fields" });
  }

  try {
    const query = `
            INSERT INTO weatherdata (sunshineduration, timestamp, temperature, humidity, rainfall, windspeed, cloudcover, forecast, weatherid)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
        `;
    await pool.query(query, [
      sunshineduration,
      timestamp,
      temperature,
      humidity,
      rainfall,
      windspeed,
      cloudcover,
      forecast,
      weatherid,
    ]);
    res.status(200).json({ status: "success" });
  } catch (err) {
    console.error(err.message);
    res.status(500).json({ error: "Database insertion failed" });
  }
});

//details for card 1
app.get('/api/card1', async (req, res) => {
    try {
      const sensorResult = await pool.query(
        `SELECT humidity, soilmoisture, temperature, distance
         FROM sensordata
         ORDER BY timestamp DESC
         LIMIT 1`
      );
  
      const weatherResult = await pool.query(
        `SELECT windspeed, cloudcover
         FROM weatherdata
         ORDER BY timestamp DESC
         LIMIT 1`
      );
  
      if (sensorResult.rows.length === 0 || weatherResult.rows.length === 0) {
        return res.status(404).json({ error: 'No data found' });
      }
  
      const sensorData = sensorResult.rows[0];
      const weatherData = weatherResult.rows[0];
  
      const windspeedKmh = parseFloat(weatherData.windspeed) * 3.6;
  
      return res.status(200).json({
        status: 'success',
        data: {
          humidity: parseFloat(sensorData.humidity), // Double as float
          soilmoisture: parseFloat(sensorData.soilmoisture), // Double as float
          temperature: parseFloat(sensorData.temperature), // Double as float
          windspeed: windspeedKmh.toFixed(2), // Float as string with 2 decimals
          cloudcover: `${parseInt(weatherData.cloudcover, 10)}%`, // Integer as string with %
          distance: sensorData.distance, // String
        },
      });
    } catch (error) {
      console.error('Error fetching sensor data:', error.message);
      return res.status(500).json({ error: 'Internal server error' });
    }
  });
  

//irrigation schedule data
app.get('/api/irrigationschedule/card2', async (req, res) => {
    try {
      const latestResult = await pool.query(
        `SELECT starttime, endtime, watervolume, duration
         FROM irrigationschedule
         ORDER BY starttime DESC
         LIMIT 1`
      );
  
      const countResult = await pool.query(
        `SELECT COUNT(DISTINCT DATE(starttime)) AS watering_count
         FROM irrigationschedule`
      );
  
      if (latestResult.rows.length === 0) {
        return res.status(404).json({ error: 'No irrigation data found' });
      }
  
      const latestIrrigation = latestResult.rows[0];
      const wateringCount = countResult.rows[0].watering_count;
  
      return res.status(200).json({
        status: 'success',
        data: {
          latestIrrigation: {
            starttime: latestIrrigation.starttime.toISOString(), // ISO string
            endtime: latestIrrigation.endtime.toISOString(), // ISO string
            watervolume: parseFloat(latestIrrigation.watervolume), // Double as float
            duration: parseInt(latestIrrigation.duration, 10), // Integer
          },
          wateringCount: parseInt(wateringCount, 10), // Integer
        },
      });
    } catch (error) {
      console.error('Error fetching irrigation data:', error.message);
      return res.status(500).json({ error: 'Internal server error' });
    }
  });
  

// Start the server
const port = process.env.PORT || 3000; // Use the PORT environment variable or default to 3000
app.listen(port, () => {
  console.log(`Server is up and running on port ${port}`);
});

// Optional: Export the app for testing purposes
module.exports = app;
