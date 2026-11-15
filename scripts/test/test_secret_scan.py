"""scripts/secret_scan.py: nothing that looks like a secret gets into a pull request.

The repo is public, and a live API key was once left in a notebook. The
scanner reads what a pull request adds (files, commit messages, title and
description) and fails on key-shaped text. Every fake secret below is put
together from pieces, so this file does not trip the scanner itself.
"""

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

import secret_scan  # noqa: E402

A20 = "Ab3dEf6hIj9kLmN2pQr5"  # 20 mixed characters, the shape of a real key body


def fake(*parts):
    return "".join(parts)


SECRETS = {
    "private key": fake("-----BEGIN ", "RSA PRIVATE", " KEY-----"),
    "OpenSSH private key": fake("-----BEGIN ", "OPENSSH PRIVATE", " KEY-----"),
    "AWS access key": fake("AKIA", "IOSFODNN7", "EXAMPLE"),
    "GitHub token": fake("ghp_", A20, A20[:16]),
    "GitHub fine-grained token": fake("github_pat_", A20, "_", A20, A20[:12]),
    "Google API key": fake("AIza", "Sy", A20, A20[:13]),
    "Slack token": fake("xoxb-", "123456789012-", A20),
    "Hugging Face token": fake("hf_", A20, A20[:14]),
    "API key assigned in code": fake('api_key="', A20, '"'),
    "API key in YAML": fake("roboflow_api_key: '", A20, "'"),
    "password assigned in code": fake('PASSWORD = "', "c0rrect-h0rse-battery", '"'),
    "token in JSON": fake('"access_token": "', A20, A20, '"'),
    "URL with a password": fake("https://deploy:", "s3cretPassw0rd", "@example.org/repo.git"),
    # A notebook cell as .ipynb JSON stores it: the quotes are escaped. This is the leak the scan was written for.
    "API key in a notebook cell": fake('    "rf = Roboflow(api_key=\\"', A20, '\\")\\n",'),
    "API key in a notebook cell, single quotes": fake("    \"rf = Roboflow(api_key='", A20, "')\\n\","),
    "all-capitals key": fake('api_key="', "ABCD1234EFGH5678IJKL", '"'),
}


@pytest.mark.parametrize("kind", sorted(SECRETS))
def test_secret_shapes_are_found_and_never_printed(kind):
    line = f"some text {SECRETS[kind]} more text"
    found = secret_scan.secret_findings(line)
    assert found, kind
    for item in found:
        assert SECRETS[kind] not in item  # the finding must not repeat the secret
        assert A20 not in item and "ABCD1234EFGH5678IJKL" not in item


@pytest.mark.parametrize(
    "line",
    [
        "api_key = os.environ['ROBOFLOW_API_KEY']",
        'api_key = "<your key here>"',
        'api_key: ""',
        'password = "changeme"',
        'token = "xxxxxxxxxxxxxxxxxxxxxxxx"',
        'api_key="${{ secrets.ROBOFLOW_API_KEY }}"',
        'api_key = "YOUR_API_KEY_GOES_HERE_OK"',
        "key from an environment variable or Colab secrets, never the repo",
        "sha256:3d3c42e5aac5ba805825da76410c181273ba90b1",
        "- uses: actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1 # v7.0.1",
        "git clone https://github.com/Purdue-NDS-RND/ARGUS_2027.git ~/ARGUS_2027",
        "ros2 launch mavros apm.launch fcu_url:=udp://:14551@",
        'camera_topic: "/camera/image"',
        "target_latitude: -35.3631271",
        'secret_findings("AKIA" + "X" * 16)',
        "the token is read-only here",
        'PASSWORD_ENV = "ARGUS_NIGHTLY_PASSWORD"',
        'token_variable = "GH_TOKEN_V2_READONLY"',
        '    "api_key = os.environ[\\"ROBOFLOW_API_KEY\\"]\\n",',
    ],
)
def test_ordinary_lines_are_not_flagged(line):
    assert secret_scan.secret_findings(line) == []


# --- on a repository ----------------------------------------------------------------------


def git(repo, *args):
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True, text=True).stdout


@pytest.fixture
def repo(tmp_path):
    """A repo with origin/main and a branch on top of it, as scripts/pr_checks.py expects."""
    repo = tmp_path / "repo"
    repo.mkdir()
    git(repo, "init", "-q", "-b", "main")
    git(repo, "config", "user.email", "dev@example.org")
    git(repo, "config", "user.name", "Dev")
    (repo / "README.md").write_text("hello\n")
    git(repo, "add", "README.md")
    git(repo, "commit", "-q", "-m", "start")
    git(repo, "update-ref", "refs/remotes/origin/main", "HEAD")
    git(repo, "checkout", "-q", "-b", "sim/feature")
    return repo


def scan(repo, **env):
    lines = []

    class Out:
        def write(self, text):
            lines.append(text)

    code = secret_scan.run(repo, env, out=Out())
    return code, "".join(lines)


def test_clean_branch_passes(repo):
    (repo / "notes.md").write_text("api_key = os.environ['KEY']\n")
    code, output = scan(repo)
    assert code == 0 and "PASS secrets" in output


def test_secret_in_an_added_line_fails_and_names_the_file_and_line(repo):
    (repo / "train.py").write_text("import os\n" + SECRETS["API key assigned in code"] + "\n")
    code, output = scan(repo)
    assert code == 1
    assert "FAIL secrets" in output and "train.py:2" in output
    assert A20 not in output


def test_secret_in_a_committed_file_and_in_a_commit_message_fails(repo):
    (repo / "deploy.sh").write_text(f"echo {SECRETS['GitHub token']}\n")
    git(repo, "add", "deploy.sh")
    git(repo, "commit", "-q", "-m", "add deploy")
    assert scan(repo)[0] == 1
    git(repo, "rm", "-q", "deploy.sh")
    git(repo, "commit", "-q", "-m", f"remove the token {SECRETS['GitHub token']}")
    code, output = scan(repo)
    assert code == 1 and "commit" in output  # deleting the file does not un-publish the message


def test_secret_already_on_main_is_not_this_pull_requests_problem(repo):
    git(repo, "checkout", "-q", "main")
    (repo / "old.py").write_text(SECRETS["AWS access key"] + "\n")
    git(repo, "add", "old.py")
    git(repo, "commit", "-q", "-m", "old")
    git(repo, "update-ref", "refs/remotes/origin/main", "HEAD")
    git(repo, "checkout", "-q", "sim/feature")
    git(repo, "merge", "-q", "main")
    (repo / "new.md").write_text("nothing to see\n")
    assert scan(repo)[0] == 0


def test_pull_request_title_and_description_are_scanned(repo):
    assert scan(repo, PR_TITLE="Add the uploader", PR_BODY="Tested with " + SECRETS["Hugging Face token"])[0] == 1
    assert scan(repo, PR_TITLE="Add the uploader", PR_BODY="Tested with the key from the environment")[0] == 0


def test_hygiene_waiver_in_the_description_lets_a_false_positive_through(repo):
    (repo / "fixture.txt").write_text(SECRETS["AWS access key"] + "\n")
    code, output = scan(repo, PR_TITLE="Add a fixture", PR_BODY="[hygiene: waived the key is AWS's documented example key]")
    assert code == 0 and "WAIVED secrets" in output
    assert scan(repo, PR_TITLE="Add a fixture", PR_BODY="[hygiene: waived]")[0] == 1  # a waiver needs a reason


def test_push_to_main_is_not_applicable_and_no_origin_main_is_a_skip(repo):
    assert scan(repo, GITHUB_EVENT_NAME="push") == (0, "secret_scan: n/a (push to main)\n")
    git(repo, "update-ref", "-d", "refs/remotes/origin/main")
    code, output = scan(repo)
    assert code == 3 and output.startswith("SKIP:")


def test_command_line_runs_on_this_repo():
    done = subprocess.run([sys.executable, str(ROOT / "scripts" / "secret_scan.py")], capture_output=True, text=True, cwd=ROOT)
    assert done.returncode in (0, 3), done.stdout + done.stderr  # 3: a checkout without origin/main
    assert "secrets" in done.stdout or "SKIP" in done.stdout


def test_check_sh_runs_the_scan():
    text = (ROOT / "scripts" / "check.sh").read_text()
    assert "scripts/secret_scan.py" in text
