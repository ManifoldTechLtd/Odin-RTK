# DevContainer 使用说明

## 1. 构建镜像

在项目根目录下执行：

```bash
docker build --pull --rm -f '.devcontainer/Dockerfile' -t 'rtkfusion:ros2_humble-amd64-1.1.0' '.devcontainer'
```

## 2. 启动容器

```bash
docker compose -f '.devcontainer/docker-compose_ROS2Humble.yml' up -d
```

## 注意事项

- **不要**使用 `--build` 参数启动，否则 docker compose 会使用独立的 BuildKit 缓存重新构建，可能因镜像源包版本更新导致 404 失败。
- 如需重新构建镜像，请重复步骤 1，然后再执行步骤 2。