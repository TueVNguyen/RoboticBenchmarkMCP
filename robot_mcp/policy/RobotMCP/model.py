"""Phía server của policy RobotMCP: chính là gateway (robot_mcp/gateway.py).

setup_policy_server.py của XPolicyLab nạp `XPolicyLab.policy.RobotMCP.model.Model`; container
gateway có `robot_mcp` trong PYTHONPATH. Container sim không bao giờ nạp file này.
"""

from robot_mcp.gateway import Gateway as Model  # noqa: F401
