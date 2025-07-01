from uipc.backend import Buffer
from uipc.backend import BufferView
import torch as pytorch

class TorchBuffer:
    @staticmethod
    def _t2b(tensor:pytorch.Tensor) -> BufferView:
        return BufferView(
            handle=tensor.contiguous().data_ptr(), 
            element_offset=0,
            element_count=tensor.numel(),
            element_size=tensor.element_size(),
            element_stride=tensor.element_size(),
            backend_name=str(tensor.device))

    def __init__(self, size: int, dtype: pytorch.dtype, device: pytorch.device = pytorch.device("cpu")):
        self._tensor = pytorch.empty(size, dtype=dtype, device=device)
        self._buffer = Buffer(
            resize_func=lambda size: self.resize(size),
            get_buffer_view_func=lambda: TorchBuffer._t2b(self._tensor)
        )

    def resize(self, newsize: int):
        '''
        Resize the tensor to the given size.
        '''
        if self._tensor.numel() != newsize:
            self._tensor.resize_(newsize)

    def buffer_view(self) -> BufferView:
        '''
        Get the buffer view of the tensor.
        '''
        return TorchBuffer._t2b(self._tensor)

    def buffer(self) -> Buffer:
        '''
        Get the buffer of the tensor.
        '''
        return self._buffer
    
    def torch(self) -> pytorch.Tensor:
        '''
        Get the torch tensor.
        '''
        return self._tensor

def buffer_view(tensor: pytorch.Tensor) -> BufferView:
    '''
    Create a buffer view from a pytorch tensor.
    '''
    return TorchBuffer._t2b(tensor)

def buffer(size: int, dtype: pytorch.dtype, device: pytorch.device = pytorch.device("cpu")) -> TorchBuffer:
    '''
    Create a TorchBuffer with the given size, dtype, and device.
    '''
    return TorchBuffer(size, dtype, device)
