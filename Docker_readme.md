# Containerization with Docker

## Creating the Docker image
### Dockerfile
Dockerfile in the root of this repo contains the recipe
to build the Docker image.
- Build the Dockerfile with Docker to create a Docker image

Build Docker image:
```
docker build . -t ttamr
```

## Running a Docker container from the created image
You have to first authorize docker to use the display with
```
xhost +local:docker
```
Next, run the docker container
```
docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host --privileged ttamr bash
```
Removing the bash command at the end makes it start the simulation demo.

For the physical demo, from within the container, follow the startup instructions as in Demo_startup.md.