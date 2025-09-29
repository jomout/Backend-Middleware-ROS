# Backend Software Engineer Assignment

This repository contains the implementation of the assignment divided into the required parts.

---

## Repository Structure

### Part 1 – Backend Gateway Task

Located in: [`backend-gateway`](./backend-gateway) folder with supporting database definitions in [`database`](./database).

- Implements the API gateway using **Next.js** and a Backend-as-a-Service (BaaS).  
- Includes device/task stack collections, protected API endpoints, and Dockerization.  
- Read Part 1  [README](./backend-gateway/README.md).

### Part 2 – Robotics Middleware "Brain" Task

Located in: [`robotics-middleware`](./robotics-middleware) folder.

- Implements the middleware service using **Python/FastAPI** to simulate robot task execution.  
- Listens for task stack events, processes tasks sequentially with simulated delays, and updates statuses.  
- Includes Dockerization and environment-based configuration.  
- Read Part 2 [README](./robotics-middleware/README.md).

### Part 3 – Code Review & Debugging

Located in: [`Part-3`](./Part-3) folder.

- Contains the analysis and corrected version of the reducer function.  
- Read Part 3 [README](./Part-3/README.md).

### Part 4 – ROS 2 Communication Integration

Located in: [`robotics-middleware`](./robotics-middleware) alongside with a simple implementation of a robot [`ros_robot`](./ros_robot) folders.

- Extends Part 2 by integrating **ROS 2 communication** (publishers/subscribers) with simulated or real nodes.  
- Middleware updates task stack status based on ROS 2 feedback.  
- Read Part 4 [README](./robotics-middleware/README.ROS.md).

---

## Running the Projects

Each part comes with its own `Dockerfile` and `docker-compose.yml`.  
See individual **READMEs** for instructions.

---

## Notes

- Parts are split into separate projects/folders as requested.  
- Part 4 lives inside the Robotics Middleware project.
- Each part’s **README** contains detailed setup, usage, and design notes.
