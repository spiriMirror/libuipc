from uipc.backend import Buffer
from uipc.backend import BufferView
import warp as pywarp
from typing import Any
import warp.types as wtypes

class WarpBuffer:
    @staticmethod
    def element_size(array: pywarp.array) -> int:
        '''
        Returns the size of an element in bytes.
        '''
        return wtypes.type_size_in_bytes(array.dtype)
    
    @staticmethod
    def element_stride(array: pywarp.array) -> int:
        '''
        Returns the stride of an element in bytes.
        '''
        return array.strides[0] if array.strides else BufferUtils.element_size(array)
    
    @staticmethod
    def _a2b(a: pywarp.array) -> BufferView:
        assert a.ndim == 1, "Only 1D arrays are supported, got {}D".format(a.ndim)
        ptr:int = 0
        if a.ptr is not None:
            ptr = a.ptr
        return BufferView(
            handle=ptr,
            element_offset=0,
            element_count=a.size,
            element_size=WarpBuffer.element_size(a),
            element_stride=WarpBuffer.element_stride(a),
            backend_name=str(a.device)
        )
    
    def __init__(self, size: int, dtype: Any, device: str = 'cpu'):
        '''
        Initialize a WarpBuffer with the given size, dtype, and device.
        '''
        self._array = pywarp.empty(size, dtype=dtype, device=device)
        self._buffer = Buffer(
            resize_func=self.resize,
            get_buffer_view_func=self.buffer_view
        )
    
    def resize(self, newsize: int) -> pywarp.array:
        '''
        Resize the array to the given size.
        '''
        if self._array.size != newsize:
            self._array = pywarp.empty(newsize, dtype=self._array.dtype, device=self._array.device)
    
    def buffer_view(self) -> BufferView:
        '''
        Get the buffer view of the array.
        '''
        return WarpBuffer._a2b(self._array)
    
    def buffer(self) -> Buffer:
        '''
        Get the buffer of the array.
        '''
        return self._buffer
    
    def warp(self) -> pywarp.array:
        '''
        Get the Warp array.
        '''
        return self._array

def buffer_view(array: pywarp.array) -> BufferView:
    '''
    Create a buffer view from a Warp array.
    '''
    return WarpBuffer._a2b(array)

def buffer(size: int, dtype: Any, device: str = 'cpu') -> WarpBuffer:
    '''
    Create a WarpBuffer with the given size, dtype, and device.
    '''
    return WarpBuffer(size, dtype, device)