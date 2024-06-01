import cv2
import time
import numpy as np
import pyrealsense2 as rs


def configure_stream(pipeline):
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)


def capture_images(pipeline, num_images, interval):
    for i in range(num_images):
        # Wait for the next frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Convert the frame to a numpy array
        image = np.asanyarray(color_frame.get_data())

        time.sleep(3)
        # Save the image
        filename = f"image_{i+20}.png"
        cv2.imwrite(filename, image)
        print(f"Image {i+1} captured and saved as {filename}")

        # Wait for the specified interval
        # time.sleep(interval)


if __name__ == "__main__":
    # Configure the RealSense pipeline
    pipeline = rs.pipeline()
    configure_stream(pipeline)

    # Number of images to capture
    num_images = 15

    # Time interval between captures (in seconds)
    interval = 3

    try:
        # Capture images
        capture_images(pipeline, num_images, interval)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Stop the pipeline
        pipeline.stop()
