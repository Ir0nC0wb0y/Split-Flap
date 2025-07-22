# Split Flap Master

## Development
Website
```shell
$ docker pull nginx:1-alpine
$ docker run --rm -p 8000:80 -v "$PWD/data/index.html:/usr/share/nginx/html/index.html:ro" -v "$PWD/basic-api-mock/v1:/usr/share/nginx/html/v1:ro" nginx:1-alpine
```

Then open http://localhost:8000.
