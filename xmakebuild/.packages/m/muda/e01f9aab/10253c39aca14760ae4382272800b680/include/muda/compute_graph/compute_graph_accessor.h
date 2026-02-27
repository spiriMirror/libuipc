#pragma once
#include <cuda_runtime.h>
#include <muda/compute_graph/compute_graph_fwd.h>
#include <muda/graph/kernel_node.h>
#include <muda/graph/memory_node.h>
#include <muda/graph/event_node.h>
namespace muda
{
namespace details
{
    // allow devlopers to access some internal function
    class ComputeGraphAccessor
    {
        friend class ComputeGraph;
        ComputeGraph& m_cg;
        template <typename T>
        using S = std::shared_ptr<T>;

      public:
        ComputeGraphAccessor();

        ComputeGraphAccessor(ComputeGraph& graph);
        ComputeGraphAccessor(ComputeGraph* graph);

        /************************************************************************************
        * 
        *                              Graph Add/Update node API
        * 
        * Automatically add or update graph node by parms (distincted by ComputeGraphPhase)
        * 
        *************************************************************************************/
        template <typename T>
        void set_kernel_node(const S<KernelNodeParms<T>>& kernelParms);
        void set_memcpy_node(void* dst, const void* src, size_t size_bytes, cudaMemcpyKind kind);
        void set_memcpy_node(const cudaMemcpy3DParms& parms);
        void set_memset_node(const cudaMemsetParams& parms);
        void set_event_record_node(cudaEvent_t event);
        void set_event_wait_node(cudaEvent_t event);
        void set_capture_node(cudaGraph_t sub_graph);

        /************************************************************************************
        * 
        *                             Current State Query API
        * 
        *************************************************************************************/
        auto current_closure() const
            -> const std::pair<std::string, ComputeGraphClosure*>&;
        auto current_closure() -> std::pair<std::string, ComputeGraphClosure*>&;
        template <typename T>
        T*                          current_node();
        const ComputeGraphNodeBase* current_node() const;
        ComputeGraphNodeBase*       current_node();
        cudaStream_t                current_stream() const;
        cudaStream_t                capture_stream() const;

        bool is_topo_built() const;

        /************************************************************************************
        * 
        *                             Current State Check API
        * 
        *************************************************************************************/
        void check_allow_var_eval() const;
        void check_allow_node_adding() const;

      private:
        friend class muda::ComputeGraphVarBase;
        void set_var_usage(VarId id, ComputeGraphVarUsage usage);

        template <typename T>
        void add_kernel_node(const S<KernelNodeParms<T>>& kernelParms);
        template <typename T>
        void update_kernel_node(const S<KernelNodeParms<T>>& kernelParms);

        void add_memcpy_node(void* dst, const void* src, size_t size_bytes, cudaMemcpyKind kind);
        void update_memcpy_node(void* dst, const void* src, size_t size_bytes, cudaMemcpyKind kind);
        void add_memcpy_node(const cudaMemcpy3DParms& parms);
        void update_memcpy_node(const cudaMemcpy3DParms& parms);

        void add_memset_node(const cudaMemsetParams& parms);
        void update_memset_node(const cudaMemsetParams& parms);

        void add_event_record_node(cudaEvent_t event);
        void update_event_record_node(cudaEvent_t event);

        void add_event_wait_node(cudaEvent_t event);
        void update_event_wait_node(cudaEvent_t event);

        void add_capture_node(cudaGraph_t sub_graph);
        void update_capture_node(cudaGraph_t sub_graph);

        template <typename F>
        void access_graph(F&& f);

        template <typename F>
        void access_graph_exec(F&& f);

        //auto&& temp_var_usage()
        //{
        //    return std::move(m_cg.m_temp_node_info.var_usage);
        //}

        template <typename NodeType, typename F>
        NodeType* get_or_create_node(F&& f);
    };
}  // namespace details
}  // namespace muda

#include "details/compute_graph_accessor.inl"