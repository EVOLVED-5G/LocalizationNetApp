NetApp to Localize an user over the 5G core

```
pip3 install uvloop httptools uvicorn fastapi fastapi_utils evolved5g
```
Docker build:

```
docker build -f Dockerfile --target dev_stage -t <image_tag>:vx.y.z .
```

To run the Localization NetApp:

```
docker run --rm -it --name netapp --privileged --net=host <image_tag>:vlatest ros2 run localization_netapp cellid_node
```
