# Setup Continuous Intergration

- Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-repository)
- Get the image either from the [Package/Container Registery](../../container_registry) or build it from the [Dockerfile](Dockerfile)
- [Register](https://docs.gitlab.com/runner/register/) a new runner
- Change the runner [docker pull policy](https://docs.gitlab.com/runner/executors/docker.html#using-the-if-not-present-pull-policy) to `if-not-present`. This will enable to use the local image you just get. To do so, open your [runner configuration file](https://docs.gitlab.com/runner/configuration/advanced-configuration.html)
