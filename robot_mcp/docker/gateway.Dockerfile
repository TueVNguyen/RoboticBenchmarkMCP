# Image cho gateway và agent thử: image sim + websockets>=14 (policy server XPolicyLab cần, image sim
# ghim 12.0) + SDK MCP. Không cần GPU. Build không cần context:
#   docker build -t robot-mcp-gateway:v0 --build-arg BASE=robodojo:cuda12.8 - < robot_mcp/docker/gateway.Dockerfile
# Env RoboDojo chỉ được kích hoạt trong entrypoint, nên lúc build `pip` là của conda base;
# phải gọi đúng python của env.
ARG BASE=robodojo:cuda12.8
FROM ${BASE}
RUN /root/miniconda3/envs/RoboDojo/bin/python -m pip install --no-cache-dir "websockets>=14" "mcp==1.30.0" \
 && /root/miniconda3/envs/RoboDojo/bin/python -c "import mcp, websockets; print('mcp ok, websockets', websockets.__version__)"
