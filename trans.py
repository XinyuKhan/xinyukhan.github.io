import re
import sys

def replace_single_dollar(match):
    return f"$${match.group(1)}$$"

def advanced_replace(content):
    split_by_double_dollar = re.split(r'(\$\$.*?\$\$)', content, flags=re.DOTALL)
    modified_split = [
        re.sub(r'(?<!\$)\$(?!\$)(.*?)(?<!\$)\$(?!\$)', replace_single_dollar, part)
        if not part.startswith('$$') else part
        for part in split_by_double_dollar
    ]
    modified_content = ''.join(modified_split)
    return modified_content

def modify_latex_expressions(input_file_path, output_file_path):
    with open(input_file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    new_content = advanced_replace(content)
    with open(output_file_path, 'w', encoding='utf-8') as file:
        file.write(new_content)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python modify_latex.py [input_file_path] [output_file_path]")
        sys.exit(1)
    input_file_path = sys.argv[1]
    output_file_path = sys.argv[2]
    modify_latex_expressions(input_file_path, output_file_path)
