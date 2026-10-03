from smc.robots.implementations.slumi.slumi_config import ConfigSlumi
from importlib.util import find_spec

if find_spec("docker"):
    import docker
else:
    raise ImportError(
        "can't run translation container without the docker package (pip install docker)"
    )


def startDockerTranslationLayer(
    cfg: ConfigSlumi, container_name: str
) -> tuple[docker.client.DockerClient, docker.models.containers.Container]:
    docker_client = docker.from_env()
    # TODO: check if you need to build the slumi_translator image (Dockerfile is in this repo),
    # i.e. check if the image exists, do a print if not and then exit.

    ports = {
        str(cfg.port_command) + "/tcp": cfg.port_command,
        str(cfg.port_data) + "/tcp": cfg.port_data,
    }
    # TODO: map appropriate volume (examples/networked_client to /home/student/bridging_layer or whatever)
    # TODO: this works for test. but there are 2 more options:
    # 1) simulation with ros1, where you need to write a script for roscore and simulation bringup launch
    # 2) real robot commands, where you need to verify you're connected to the robot's network, and do a different bringup launch
    # scripts are in this, but ... idk i didn't finish this thougt
    translation_script_cmd = f"python3 bridging_scripts/slumi_translator_test.py --host={cfg.host} --port-command={cfg.port_command} --port-data={cfg.port_data}  --sending-frequency={cfg.sending_frequency}"
    # NOTE: python 3.8's argparse does not have --no-option because it does not have "booleanAction" yet
    # so we put it off by default (you can't do it otherwise), and you put the argument if you want it set to true.
    # this feels less hacky than using integers for booleans.
    # and yes, you need --send-commands=True , and no --send-commands=False does not work lmao
    if cfg.send_commands:
        translation_script_cmd += "--send-commands={str(cfg.send_commands)}"
    if cfg.debug_prints:
        print(
            "starting docker translation layer with the command:\n",
            translation_script_cmd,
        )
    try:
        container = docker_client.containers.get(container_name)
        container.kill()
    except (docker.errors.NotFound, docker.errors.APIError):
        print(
            "seems like the translation container is running already. i'm restarting it"
        )
    try:
        container = docker_client.containers.get(container_name)
        container.remove()
    except (docker.errors.NotFound, docker.errors.APIError):
        pass
    print(
        "TODO: verify startup bash works, go to code and enable automatic docker container startup (it works and is auto-configured with arguments!)"
    )
    translation_container = docker_client.containers.run(
        "slumi_translator:latest",
        translation_script_cmd,
        detach=True,
        ports=ports,
        network="host",  # simply does nothing on macos
        name="slumi_translator",
    )
    return docker_client, translation_container
