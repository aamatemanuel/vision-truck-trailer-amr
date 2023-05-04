import yaml

with open('example.yaml', 'r') as file:
    yaml_content = yaml.safe_load(file)

print(yaml_content.num_wheels)