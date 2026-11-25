for v in 0.6 1.0 1.4; do venv/bin/python opt.py mevcut $v --min 8; done
for v in 1.4 1.6; do venv/bin/python opt.py hizli $v --min 8; venv/bin/python opt.py guclu $v --min 8; done
