def escape_quotes(text):
    return text.replace('"', '\\"')

def process_file(filename):
    with open(filename, 'r') as file:
        return file.read().strip()

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
    {js}
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