#!/usr/bin/env python3
"""Builds data/ (the LittleFS image) from the page sources in web/.

2026-10-22: a browser loading Annabelle's page opened 6 connections at once (page, Bootstrap CSS,
SVG, two scripts, four logos) and ran her out of memory. Everything now goes into one gzipped
data/index.html.gz - the server sends the .gz with Content-Encoding: gzip - so a page load is one
request plus the page's own JSON polls.

Edit the files in web/, run  python3 build_web.py , then upload data/ to LittleFS.
Uploading data/ replaces the whole file system, including the queue for the Pi and the reset log.
"""
import base64
import gzip
import pathlib
import shutil

ROOT = pathlib.Path(__file__).resolve().parent
WEB = ROOT / "web"
DATA = ROOT / "data"


def text(name):
    return (WEB / name).read_text(encoding="utf-8")


def data_uri(path, mime):
    return "data:%s;base64,%s" % (mime, base64.b64encode(path.read_bytes()).decode("ascii"))


def replace_once(html, old, new):
    if html.count(old) != 1:
        raise SystemExit("index.html: expected exactly one %r" % old)
    return html.replace(old, new)


def main():
    html = text("index.html")
    html = replace_once(html, '<link rel="stylesheet" href="assets/bootstrap/css/bootstrap.min.css">',
                        "<style>\n" + text("bootstrap.min.css") + "\n</style>")
    html = replace_once(html, 'src="assets/img/Annabelle.svg"',
                        'src="' + data_uri(WEB / "Annabelle.svg", "image/svg+xml") + '"')
    logos = ",\n".join('    "%s": "%s"' % (p.stem, data_uri(p, "image/png"))
                       for p in sorted((WEB / "logos").glob("*.png")))
    html = replace_once(html, '<script src="assets/js/telepathons.js"></script>',
                        "<script>\nwindow.KIND_LOGO_DATA = {\n" + logos + "\n};\n</script>\n"
                        "<script>\n" + text("telepathons.js") + "\n</script>")
    html = replace_once(html, '<script src="assets/js/settings.js"></script>',
                        "<script>\n" + text("settings.js") + "\n</script>")

    if DATA.exists():
        shutil.rmtree(DATA)
    DATA.mkdir()
    raw = html.encode("utf-8")
    packed = gzip.compress(raw, compresslevel=9, mtime=0)
    (DATA / "index.html.gz").write_bytes(packed)
    print("data/index.html.gz: %d KB (%d KB before gzip)" % (len(packed) // 1024, len(raw) // 1024))


if __name__ == "__main__":
    main()
