const serverless = require('serverless-http');
const app = require('../index'); // Adjust the path if your Express app is not in the root directory

module.exports = serverless(app);
