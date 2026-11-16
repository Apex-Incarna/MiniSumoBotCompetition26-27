# From Google AI overview

import cv2
from flask import Flask, Response

app = Flask(__name__)

# 0 is usually the built-in or primary USB webcam
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        
        # Encode the frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
            
        frame_bytes = buffer.tobytes()
        
        # Yield the frame in MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    # Embeds the stream directly on the main localhost page
    return '''
    <html>
        <head><title>Live Camera Feed</title></head>
        <body style="margin:0; background:#111; display:flex; justify-content:center; align-items:center; height:100vh;">
            <img src="/video_feed" style="max-width:100%; max-height:100%;">
        </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # threaded=True prevents the server from blocking if multiple tabs open the stream
    app.run(host='0.0.0.0', port=5000, threaded=True)
