import os
def create(filename):
    h_filename = filename + '.h'
    detail_filename = 'details/' + filename + '.inl'
    if not os.path.exists(h_filename):
        f = open(h_filename, 'w')
        content = f'''#pragma once


    
#include "details/{filename}.inl"
'''
        f.write(content)
        f.close()
    else:
        print(f'{h_filename} already exists')
    
    f = open('../linear_system.h', 'a')

    if not os.path.exists(detail_filename):
        f = open(detail_filename, 'w')
        f.write('')
        f.close()
    else:
        print(f'{detail_filename} already exists')

def create_files(filenames:list[str]):
    for filename in filenames:
        create(filename)
    
    f = open('../linear_system.h', 'w')
    f.write('#pragma once\n\n')
    for filename in filenames:
        f.write(f'#include <muda/ext/linear_system/{filename}.h>\n')
    f.close()
    f = open('../linear_system.h', 'r')
    print(f.read())


create_files(
    [  
        'linear_system_context',
        
        'vector_format_converter',
        'matrix_format_converter',

        'device_dense_matrix',
        'dense_matrix_view',
        'dense_matrix_viewer',

        'device_dense_vector',
        'dense_vector_view',
        'dense_vector_viewer',

        'device_triplet_matrix',
        'triplet_matrix_view',
        'triplet_matrix_viewer',

        'device_doublet_vector',
        'doublet_vector_view',
        'doublet_vector_viewer',

        'device_bcoo_matrix',
        'bcoo_matrix_view',
        'bcoo_matrix_viewer',

        'device_bcoo_vector',
        'bcoo_vector_view',
        'bcoo_vector_viewer',

        'device_bsr_matrix',
        'bsr_matrix_view',

        'device_csr_matrix',
        'csr_matrix_view',
    ]
)


