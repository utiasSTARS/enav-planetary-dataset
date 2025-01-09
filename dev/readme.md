# dev workflow

From repo's root directory:

1. Create .env file with path to dataset:

```sh
echo "ENAV_DATASET_DIR='/path/to/dataset/dir'" > .env
```

2. Create dev image & start container (-d runs it in the background):

```sh
docker compose -f dev/docker-compose.dev.yml --env-file .env up -d
```

3. Open container in vscode: Bottom left: `Open a Remove Window` > `Attach to a running container`

Useful container commands:

```sh
# Check running container name
docker container ls

# Open the running container in a separate shell
docker container exec -it [name] bash

# Stop a running container
docker container stop [name]

# Remove a container
docker container rm [name]
```

Useful image commands:

```sh
# Check image name
docker images

# Remove image
docker image rm [name]
```

## Re-build & push new main image

1. Remove older images, if any:

```sh
docker image rm enav
```

2. Regenerate a new image:

```sh
docker build -t enav --no-cache .
```

3. Update image name & tag to point to olamarre's dockerhub repo:

```sh
docker tag enav:latest olamarre/enav:latest
```

4. Push changes to dockerhub

```sh
docker push olamarre/enav
```

5. Erase local images & test installation with docker-compose file:

```sh
docker image rm enav olamarre/enav

docker compose run --rm enav
```