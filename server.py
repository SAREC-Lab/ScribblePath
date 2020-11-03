#!/usr/bin/env python2.7

from http.server import BaseHTTPRequestHandler, HTTPServer
import socketserver
import json

PORT = 8080

class Handler(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        pass

    def do_POST(self):
        content_length = int(self.headers['Content-Length']) # Size of Data
        data = self.rfile.read(content_length).decode('utf-8') # Data
        coords = json.loads(data)
        print(coords)

        self._set_response()
        self.wfile.write("Recieved".encode('utf-8'))

def run(server_class=HTTPServer, handler_class=Handler, port=PORT):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print('Starting server...\n')
    try:
        print('PORT: ' + str(PORT))
        print('URL: localhost:' + str(PORT))
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    print('Stopping server...\n')

if __name__ == '__main__':
    run()