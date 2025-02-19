import re

def escape_quotes(text):
    return text.replace('"', '\\"')

def minify_js(js_content):
    """Remove whitespace and newlines from JavaScript while preserving functionality."""
    # Remove comments
    js_content = re.sub(r'//.*?\n|/\*.*?\*/', '', js_content, flags=re.DOTALL)
    
    # Split on function boundaries to keep functions together
    functions = js_content.split('function')
    minified = []
    
    for i, func in enumerate(functions):
        if i == 0:  # First part might be empty or contain non-function code
            if func.strip():
                minified.append(func.strip())
            continue
            
        # Reconstruct function and remove unnecessary whitespace
        func = 'function' + func
        func = ' '.join(func.split())  # Collapse whitespace
        # Ensure proper spacing around arrow functions
        func = func.replace('=>', ' => ')
        # Remove spaces around special characters while preserving arrow functions
        for char in ['{', '}', '(', ')', ',', ';', '=', '+', '-', '*', '/', ':', '?']:
            if char != '=':  # Skip = to preserve =>
                func = func.replace(f' {char} ', char)
                func = func.replace(f' {char}', char)
                func = func.replace(f'{char} ', char)
        minified.append(func)
    
    return ''.join(minified)

def process_file(filename):
    with open(filename, 'r') as file:
        content = file.read().strip()
        if filename.endswith('.js'):
            return minify_js(content)
        return content

def main():
    # Read the files
    html = process_file('index.html')
    css = process_file('styles.css')
    js = process_file('script.js')

    # Insert CSS and JS into HTML
    html_parts = html.split('</head>')
    html_with_assets = html_parts[0] + f'''
    <style>
    {css}
    </style>
    <script>
    ''' + js + '''
    </script>
    </head>''' + html_parts[1]

    # Convert to client.print statements
    output_lines = []
    for line in html_with_assets.split('\n'):
        line = line.strip()
        if line:
            escaped_line = escape_quotes(line)
            output_lines.append(f'client.print("{escaped_line}");')

    # Write to output file
    with open('output.txt', 'w') as f:
        f.write('\n'.join(output_lines))

if __name__ == "__main__":
    main() 