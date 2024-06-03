import socket
import cv2
import numpy as np
import torch
import torch.jit as jit
import pyrealsense2 as rs
from datetime import datetime
import time
import sys

# Constants
HOST = "192.168.1.120"  # IP of the control board
PORT = 12125  # Port to send data to
RESNET_MODEL_PATH = "/home/ysc/udp/resnet/depth_lidar_model_20240528-142228_300.pt"

FRAME_WIDTH_ = 848 // 8
FRAME_HEIGHT_ = 480 // 8
OUT_DIMEN_ = 11

model = None

def load_model():
    global model
    try:
        # Deserialize the ScriptModule from a file using torch.jit.load().
        model = jit.load(RESNET_MODEL_PATH)
        # Check if CUDA is available and move tensor to GPU
        if torch.cuda.is_available():
            model.to('cuda')
            print("Using CUDA")
        else:
            print("Using CPU")
    except RuntimeError as e:
        print("Error loading the model.")
        sys.exit(1)

def resnet_infer(frame):
    tensor = torch.from_numpy(frame.astype(np.float32)).clone()
    tensor = tensor.unsqueeze(0).repeat(1, 3, 1, 1)  # Repeat the tensor to match the input size of the model
    if torch.cuda.is_available():
        tensor = tensor.to('cuda')
    with torch.no_grad():
        output = model.forward(tensor).squeeze().cpu()
    return output

def send_depth_data():
    load_model()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver_address = (HOST, PORT)

    pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.depth, FRAME_WIDTH_, FRAME_HEIGHT_, rs.format.z16, 30)
    pipeline.start()

    last_time = datetime.now()
    frame_count = 0

    while True:
        frames = pipeline.wait_for_frames()  # Wait for a new frame
        depth = frames.get_depth_frame()  # Get the depth frame

        # Convert the depth frame to a NumPy array
        depth_image = np.asanyarray(depth.get_data(), dtype=np.uint16)

        # Convert depth from millimeters to meters
        depth_image = depth_image.astype(np.float32) / 1000

        # Clip the values between 0 and 6 meters
        np.clip(depth_image, 0, 6, out=depth_image)

        # Resize the depth image using OpenCV
        resized_depth_image = cv2.resize(depth_image, (FRAME_WIDTH_, FRAME_HEIGHT_), interpolation=cv2.INTER_LINEAR)

        # Convert the resized NumPy array to an Eigen matrix (using NumPy)
        eigen_matrix = resized_depth_image

        output = resnet_infer(eigen_matrix)

        # Check output dimensions
        if output.numel() != OUT_DIMEN_:
            print(f"Unexpected output size from resnet_infer. Expected {OUT_DIMEN_}, got {output.numel()}")
            return  # or handle error appropriately

        eigen_vector = output.numpy()
        
        # print(f"Output: {eigen_vector}")

        # Serialize and send data (consider compression or proper serialization here)
        data = eigen_vector.tobytes()

        sock.sendto(data, receiver_address)
        frame_count += 1

        # Timer and frequency calculation
        now = datetime.now()
        duration = (now - last_time).total_seconds()
        if duration >= 1:
            print(f"Sending frequency: {frame_count} frames/sec")
            frame_count = 0
            last_time = now

        time.sleep(0.001)  # Send at 10Hz

def receive_depth_data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', PORT))

    while True:
        data, address = sock.recvfrom(1024)
        print(f"Received: {data.decode()} from {address}")

def main():
    if len(sys.argv) != 2:
        print("Usage: depth_transceiver <send|receive>")
        sys.exit(1)
    mode = sys.argv[1]

    if mode == "send":
        send_depth_data()
    elif mode == "receive":
        receive_depth_data()
    else:
        print("Invalid mode. Use 'send' or 'receive'.")
        sys.exit(1)

if __name__ == "__main__":
    main()
