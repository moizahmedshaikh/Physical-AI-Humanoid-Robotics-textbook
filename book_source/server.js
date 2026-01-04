const express = require('express');
const path = require('path');
const serveStatic = require('serve-static');
const history = require('connect-history-api-fallback');

const app = express();

// Serve static files from the build directory
app.use(serveStatic(path.join(__dirname, 'build')));

// Set up proxy for API requests
const { createProxyMiddleware } = require('http-proxy-middleware');

app.use('/api', createProxyMiddleware({
  target: 'http://localhost:8000', // Backend API server
  changeOrigin: true,
  pathRewrite: {
    '^/api': '', // Remove /api prefix when forwarding to backend
  },
}));

// Enable HTML5 History API
app.use(history());

// Serve static files for any other routes
app.use(serveStatic(path.join(__dirname, 'build')));

const port = process.env.PORT || 3000;
app.listen(port, () => {
  console.log(`Server running on port ${port}`);
});