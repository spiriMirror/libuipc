'''Main entry point for uipc.cli - shows available CLI tools'''
import sys
import importlib
import inspect
from pathlib import Path


def discover_cli_tools():
    '''Automatically discover CLI tools in the cli package'''
    cli_tools = []
    
    # Get the directory containing this file
    cli_dir = Path(__file__).parent
    
    # Find all Python files in the cli directory
    for py_file in cli_dir.glob('*.py'):
        # Skip __init__.py and __main__.py
        if py_file.stem in ('__init__', '__main__'):
            continue
        
        module_name = f'uipc.cli.{py_file.stem}'
        try:
            # Import the module
            module = importlib.import_module(module_name)
            
            # Check if it has a main() function
            if hasattr(module, 'main') and callable(module.main):
                # Get module docstring
                module_doc = inspect.getdoc(module) or ''
                # Get main() function docstring
                main_doc = inspect.getdoc(module.main) or ''
                # Use main docstring if available, otherwise module docstring
                description = main_doc.split('\n')[0] if main_doc else (module_doc.split('\n')[0] if module_doc else '')
                
                cli_tools.append({
                    'name': py_file.stem,
                    'module': module_name,
                    'description': description
                })
        except Exception as e:
            # Skip modules that can't be imported
            continue
    
    return sorted(cli_tools, key=lambda x: x['name'])


def main():
    '''Display available CLI tools and usage information'''
    tools = discover_cli_tools()
    
    print('=' * 60)
    print('LibUIPC CLI Tools')
    print('=' * 60)
    print()
    
    if not tools:
        print('No CLI tools found.')
        print('=' * 60)
        return
    
    print('Available commands:')
    print()
    
    for i, tool in enumerate(tools, 1):
        print(f'  {i}. {tool["name"]}')
        if tool['description']:
            print(f'     {tool["description"]}')
        print(f'     Usage: python -m {tool["module"]} [options]')
        print()

if __name__ == '__main__':
    main()

