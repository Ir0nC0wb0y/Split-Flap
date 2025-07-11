# Split Flap Master

## Development
Website
```shell
$ docker pull nginx:1-alpine
$ docker run --rm -p 8000:80 -v "$PWD/src/data:/usr/share/nginx/html:ro" nginx:1-alpine
```

Then open http://localhost:8000.
