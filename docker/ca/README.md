# Extra CA certificates for the image

Anything `*.crt` in here is installed into the image's trust store at build
time. The `.crt` files are git-ignored: they are a property of the network the
build machine sits on, not of this project.

Why it exists: the campus network intercepts TLS to some hosts (crates.io,
pypi.org -- github.com is left alone) and re-signs them with its own CA. The
host trusts that CA (`/usr/local/share/ca-certificates/KMU.crt`); a container
does not, so `cargo fetch` and `pip install` inside it fail with "self-signed
certificate in certificate chain" while `apt` (plain http) and github keep
working, which looks like anything but a certificate problem. Measured, not
guessed: `curl -v https://static.crates.io/` from the container.

    cp /usr/local/share/ca-certificates/*.crt docker/ca/     # then make build

Off campus, leave it empty; nothing changes.
