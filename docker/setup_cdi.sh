#!/bin/bash
# Generate NVIDIA CDI spec and remove MPS entries that may not exist on all hosts.
# Run once per host (as root) before starting the CUDA container.
# Usage: sudo bash docker/setup_cdi.sh

set -e

nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

python3 -c "
import os, yaml

def strip_edits(edits):
    edits['mounts'] = []
    edits['hooks'] = []

for path in ['/etc/cdi/nvidia.yaml', '/run/cdi/nvidia.yaml']:
    if not os.path.exists(path):
        continue
    with open(path) as f:
        spec = yaml.safe_load(f)
    strip_edits(spec['containerEdits'])
    for dev in spec.get('devices', []):
        strip_edits(dev.get('containerEdits', {}))
    with open(path, 'w') as f:
        yaml.dump(spec, f, default_flow_style=False)
    print(f'{path}: stripped mounts and hooks -- device nodes kept')
"
