from robo.main import greet


def test_greet_returns_expected_string() -> None:
    assert greet("tester") == "Hello, tester!"
