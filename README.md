Certainly! Here's how you can extend the README to include the instructions for publishing input queries to the `/input_query` topic:

---

# Jetson Nano LLM and NanoOWL Vision Encoder Setup

This repository provides the setup for running a Jetson-based ROS2 workspace with the NanoOWL vision encoder using TensorRT on NVIDIA Jetson devices. Follow the steps below to set up and run the environment, build the required image encoder engine, and launch the camera input example.

---

## Supported Jetson Devices

- **Jetson AGX Orin (64GB)**
- **Jetson AGX Orin (32GB)**
- **Jetson Orin NX (16GB)**
- **Jetson Orin Nano (8GB) (⚠️)**

## Supported JetPack Versions

- **JetPack 5 (L4T r35.x)**
- **JetPack 6 (L4T r36.x)**

## Requirements

- **NVMe SSD** (Recommended for fast storage and space):
  - **30GB** for `kabilankb/nano_llm_ros2_kb` container image

## Setup Instructions

### 1. Clone and Setup `jetson-containers`

Start by cloning the `jetson-containers` repository and running the installation script to set up the environment:

```bash
git clone https://github.com/dusty-nv/jetson-containers
cd jetson-containers
bash install.sh
```

### 2. Launch the Container

Run the container with your ROS2 workspace mounted:

```bash
jetson-containers run -v ~/ros2_workspace:/ros2_workspace $(kabilankb/nano_llm_ros2_kb)
```

### 3. Clone and Setup `nanoowl`

Clone the `nanoowl` repository and build the OWL-ViT vision encoder engine:

```bash
git clone https://github.com/NVIDIA-AI-IOT/nanoowl
cd nanoowl
mkdir -p data
python3 -m nanoowl.build_image_encoder_engine data/owl_image_encoder_patch32.engine
```

### 4. Launch ROS2 Node for Camera Feed

After building the engine, launch the ROS2 node that will subscribe to the camera feed from Isaac Sim:

```bash
ros2 launch ros2_nanoowl camera_input_example.launch.py thresholds:=0.5 image_encoder_engine:='src/nanoowl/data/owl_image_encoder_patch32.engine'
```

This node will receive the published Isaac Sim camera feed and process it using the OWL-ViT vision encoder.

### 5. Publish Input Query to `/input_query` Topic

From another terminal, you can publish input queries to the `/input_query` topic. This allows you to specify the objects you want to detect. You can change the query anytime while the `ros2_nanoowl` node is running.

Use the following command to publish a list of objects:

```bash
ros2 topic pub /input_query std_msgs/String 'data: "a person, a box"'
```

You can modify the object list to detect different objects, for example:

```bash
ros2 topic pub /input_query std_msgs/String 'data: "a dog, a car, a tree"'
```

The `ros2_nanoowl` node will detect and process the new query in real-time.

---
![Screenshot from 2025-02-05 19-04-39](https://github.com/user-attachments/assets/8ebd90d2-2ccf-4773-8935-794258a64464)

![Screenshot from 2025-02-05 19-03-55](https://github.com/user-attachments/assets/f877fe10-dda4-4c69-be88-7688b520f818)

![Screenshot from 2025-02-05 19-05-49](https://github.com/user-attachments/assets/2ef91b73-0279-4982-99b5-5450e89cf446)


## Additional Information

- **Container Setup**: The container will provide all the necessary dependencies for running ROS2 nodes and the OWL-ViT vision encoder.
- **Image Encoder**: The TensorRT engine for OWL-ViT is built with the `nanoowl.build_image_encoder_engine` command and stored in `data/owl_image_encoder_patch32.engine`.
- **Input Query**: The `/input_query` topic allows you to dynamically change the objects that the vision encoder detects.

## Troubleshooting

- Ensure you have enough storage space on your device, especially for models (>10GB).
- Verify that you are using a compatible Jetson device and JetPack version.

---

## License

This repository is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---
