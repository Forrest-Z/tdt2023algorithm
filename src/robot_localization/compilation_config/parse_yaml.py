# parse_yaml.py
import sys
import yaml

if len(sys.argv) != 2:
    sys.stderr.write('Usage: {} <YAML_FILE>\n'.format(sys.argv[0]))
    sys.exit(1)

yaml_file = sys.argv[1]

with open(yaml_file, 'r') as f:
    yaml_data = yaml.safe_load(f)

for key, value in yaml_data.items():
    print('{}={}'.format(key, value))
