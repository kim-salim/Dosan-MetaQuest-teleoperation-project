from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_ros_tcp_receive_path_does_not_print_every_message():
    client = (PACKAGE_ROOT / "ros_tcp_endpoint" / "client.py").read_text()
    publisher = (PACKAGE_ROOT / "ros_tcp_endpoint" / "publisher.py").read_text()
    server = (PACKAGE_ROOT / "ros_tcp_endpoint" / "server.py").read_text()

    assert "Received raw message" not in client
    assert "Publishing parsed message" not in publisher
    assert "Received raw message" not in server
    assert "list(data)" not in client
