import re

def escape_special_chars(text):
    """Escape special characters for Arduino client.print statements."""
    return (text.replace('\\', '\\\\')  # Must escape backslashes first
              .replace('"', '\\"')
              .replace('${', '\\${')     # Escape template literal expressions
    )
def minify_js(js_content):
    """Minify JavaScript while preserving functionality."""
    # Remove comments
    js_content = re.sub(r'//.*?\n|/\*.*?\*/', '', js_content, flags=re.DOTALL)
    
    # Preserve important whitespace around operators
    js_content = re.sub(r'([=<>!+\-*/%&|^])\s+', r'\1', js_content)  # Remove space after operator
    js_content = re.sub(r'\s+([=<>!+\-*/%&|^])', r'\1', js_content)  # Remove space before operator
    
    # Special handling for arrow functions
    js_content = re.sub(r'\s*=>\s*', ' => ', js_content)  # Preserve spaces around =>
    
    # Remove unnecessary whitespace while preserving needed spaces
    lines = js_content.split('\n')
    minified_lines = []
    for line in lines:
        line = line.strip()
        if line:
            # Collapse multiple spaces into single space
            line = ' '.join(line.split())
            minified_lines.append(line)
    
    return ' '.join(minified_lines)

def process_file(filename):
    """Read and process a file based on its type."""
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
    html_with_assets = (
        html_parts[0] + 
        f'\n<style>\n{css}\n</style>\n' +
        f'<script>\n{js}\n</script>\n' +
        '</head>' + 
        html_parts[1]
    )

    # Convert to client.print statements
    output_lines = []
    current_line = ''
    
    for line in html_with_assets.split('\n'):
        line = line.strip()
        if line:
            # Escape special characters
            escaped_line = escape_special_chars(line)
            
            # Handle long lines by breaking them up
            if len(escaped_line) > 120:  # Arduino has line length limits
                if current_line:
                    output_lines.append(f'client.print("{current_line}");')
                    current_line = ''
                output_lines.append(f'client.print("{escaped_line}");')
            else:
                if current_line:
                    current_line += '\\n' + escaped_line
                else:
                    current_line = escaped_line
                
                if len(current_line) > 100:  # Write out accumulated line
                    output_lines.append(f'client.print("{current_line}");')
                    current_line = ''
    
    # Write any remaining content
    if current_line:
        output_lines.append(f'client.print("{current_line}");')

    # Write to output file
    with open('output.txt', 'w') as f:
        f.write('\n'.join(output_lines))

if __name__ == "__main__":
    main() 