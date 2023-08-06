#include "ikd_Tree.h"
#include <thread>
/*
Description: ikd-Tree: an incremental k-d tree for robotic applications 
Author: Yixi Cai
email: yixicai@connect.hku.hk
*/

template <typename PointType>
KD_TREE<PointType>::KD_TREE(float delete_param, float balance_param, float box_length)
{
    // 设定平衡判断参数
    delete_criterion_param = delete_param;
    balance_criterion_param = balance_param;
    downsample_size = box_length;
    Rebuild_Logger.clear();
    termination_flag = false;
    // 初始化一系列数据互斥锁，并且创建重构树的线程
    // 这个线程相当于一直监控着树的平衡性，当打破平衡法则时就自动进行重建
    start_thread();
}

template <typename PointType>
KD_TREE<PointType>::~KD_TREE()
{
    stop_thread();
    Delete_Storage_Disabled = true;
    delete_tree_nodes(&Root_Node);
    PointVector().swap(PCL_Storage);
    Rebuild_Logger.clear();
}



template <typename PointType>
void KD_TREE<PointType>::InitializeKDTree(float delete_param, float balance_param, float box_length)
{
    Set_delete_criterion_param(delete_param);
    Set_balance_criterion_param(balance_param);
    set_downsample_param(box_length);
}

template <typename PointType>
void KD_TREE<PointType>::InitTreeNode(KD_TREE_NODE *root) // 初始化根节点
{
    root->point.x = 0.0f;
    root->point.y = 0.0f;
    root->point.z = 0.0f;
    root->node_range_x[0] = 0.0f;
    root->node_range_x[1] = 0.0f;
    root->node_range_y[0] = 0.0f;
    root->node_range_y[1] = 0.0f;
    root->node_range_z[0] = 0.0f;
    root->node_range_z[1] = 0.0f;
    root->radius_sq = 0.0f;
    root->division_axis = 0;
    root->father_ptr = nullptr;
    root->left_son_ptr = nullptr;
    root->right_son_ptr = nullptr;
    root->TreeSize = 0;
    root->invalid_point_num = 0;
    root->down_del_num = 0;
    root->point_deleted = false;
    root->tree_deleted = false;
    root->need_push_down_to_left = false;
    root->need_push_down_to_right = false;
    root->point_downsample_deleted = false;
    root->working_flag = false;
    // 初始化互斥锁 相当于 pthread_mutex_t root->push_down_mutex_lock = PTHREAD_MUTEX_INITIALIZER
    pthread_mutex_init(&(root->push_down_mutex_lock), NULL);
}

template <typename PointType>
int KD_TREE<PointType>::size()
{
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
        {
            return Root_Node->TreeSize;
        }
        else
        {
            return 0;
        }
    }
    else
    {   
        /*锁操作主要包括加锁pthread_mutex_lock()、解锁pthread_mutex_unlock()和
        测试加锁 pthread_mutex_trylock()三个，不论哪种类型的锁，都不可能被两个不
        同的线程同时得到，而必须等待解锁。对于普通锁和适应锁类型，解锁者可以是同进程
        内任何线程；而检错锁则必须由加锁者解锁才有效，否则返回EPERM；对于嵌套锁，文
        档和实现要求必须由加锁者解锁，但测试结果表明并没有这种限制，这个不同目前还没
        有得到解释。在同一进程中的线程，如果加锁后没有解锁，则任何其他线程都无法再获得锁。*/
        // pthread_mutex_trylock()语义与pthread_mutex_lock()类似，不同的是在锁已经被占据时返回 EBUSY 而不是挂起等待

        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            s = Root_Node->TreeSize;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        }
        else
        {
            return Treesize_tmp;
        }
    }
}

template <typename PointType>
BoxPointType KD_TREE<PointType>::tree_range() // 获取一个包围整个树的Box边界
{
    BoxPointType range;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
        {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
        }
        else
        {
            memset(&range, 0, sizeof(range));
        }
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
            pthread_mutex_unlock(&working_flag_mutex);
        }
        else
        {
            memset(&range, 0, sizeof(range));
        }
    }
    return range;
}

template <typename PointType>
int KD_TREE<PointType>::validnum()
{
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
            return (Root_Node->TreeSize - Root_Node->invalid_point_num);
        else
            return 0;
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            s = Root_Node->TreeSize - Root_Node->invalid_point_num;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        }
        else
        {
            return -1;
        }
    }
}

template <typename PointType>
void KD_TREE<PointType>::root_alpha(float &alpha_bal, float &alpha_del)
{
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        alpha_bal = Root_Node->alpha_bal;
        alpha_del = Root_Node->alpha_del;
        return;
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            alpha_bal = Root_Node->alpha_bal;
            alpha_del = Root_Node->alpha_del;
            pthread_mutex_unlock(&working_flag_mutex);
            return;
        }
        else
        {
            alpha_bal = alpha_bal_tmp;
            alpha_del = alpha_del_tmp;
            return;
        }
    }
}

// 如果用c++11支持的std::thread来写会不会看起来简略一些
// https://blog.csdn.net/m0_37251750/article/details/126409127
template <typename PointType>
void KD_TREE<PointType>::start_thread()
{
    // 初始一系列互斥锁
    pthread_mutex_init(&termination_flag_mutex_lock, NULL);
    pthread_mutex_init(&rebuild_ptr_mutex_lock, NULL);
    pthread_mutex_init(&rebuild_logger_mutex_lock, NULL);
    pthread_mutex_init(&points_deleted_rebuild_mutex_lock, NULL);
    pthread_mutex_init(&working_flag_mutex, NULL);
    pthread_mutex_init(&search_flag_mutex, NULL);
    // 创建重建树的线程，线程标识符指针：rebuild_thread， 运行函数：multi_thread_ptr
    pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void *)this);
    printf("Multi thread started \n");
}

template <typename PointType>
void KD_TREE<PointType>::stop_thread()  // 停止线程
{
    pthread_mutex_lock(&termination_flag_mutex_lock);
    termination_flag = true;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    // 等待重构树的线程结束
    if (rebuild_thread)
        pthread_join(rebuild_thread, NULL);
    /*销毁一个互斥锁即意味着释放它所占用的资源，且要求锁当前处于开放状态。由于在Linux中，
    互斥锁并不占用任何资源，因此LinuxThreads中的 pthread_mutex_destroy()除了检查
    锁状态以外（锁定状态则返回EBUSY）没有其他动作*/
    pthread_mutex_destroy(&termination_flag_mutex_lock);
    pthread_mutex_destroy(&rebuild_logger_mutex_lock);
    pthread_mutex_destroy(&rebuild_ptr_mutex_lock);
    pthread_mutex_destroy(&points_deleted_rebuild_mutex_lock);
    pthread_mutex_destroy(&working_flag_mutex);
    pthread_mutex_destroy(&search_flag_mutex);
}

template <typename PointType>
void *KD_TREE<PointType>::multi_thread_ptr(void *arg)
{
    // 初始化一个树指针
    KD_TREE *handle = (KD_TREE *)arg;
    handle->multi_thread_rebuild();
    return nullptr;
}

template <typename PointType>
void KD_TREE<PointType>::multi_thread_rebuild() // 构建ikd tree后，该线程一直在运行
{
    bool terminated = false;
    // 新建虚拟头结点和重建的树的新节点
    KD_TREE_NODE *father_ptr, **new_node_ptr;
    pthread_mutex_lock(&termination_flag_mutex_lock);
    // KD树初始化时termination_flag被设为false，所以后面的while循环是一直进行的
    terminated = termination_flag;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    while (!terminated)
    {   
        // The second thread will lock all incremental updates (i.e., points insert, re-insert,
        // and delete) but not queries on this sub-tree
        // 锁定了插入、重插入、删除功能，但不锁定查询功能
        pthread_mutex_lock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&working_flag_mutex);
        if (Rebuild_Ptr != nullptr)
        {
            /* Traverse and copy */
            // 如果在重建某个子树的过程中，用户对该子树有新的修改操作，例如插入、删除、属性传递等，
            // 就会将这些操作记录到Rebuild_Logger中，等完成重建后再补作业
            if (!Rebuild_Logger.empty())
            {
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }
            rebuild_flag = true;
            // 判断重建的子树的根节点是否为整棵树的实际根节点
            if (*Rebuild_Ptr == Root_Node)
            {   // 如果是的话，要记录一些根节点特有的参数，避免重建过程中造成数据丢失，因为待会树的实际根节点会被替换
                Treesize_tmp = Root_Node->TreeSize;
                Validnum_tmp = Root_Node->TreeSize - Root_Node->invalid_point_num;
                alpha_bal_tmp = Root_Node->alpha_bal;
                alpha_del_tmp = Root_Node->alpha_del;
            }
            // 重建前子树的根节点
            KD_TREE_NODE *old_root_node = (*Rebuild_Ptr);
            // 记录该子树的父节点，重建完成后需要重新接驳回去
            father_ptr = (*Rebuild_Ptr)->father_ptr;
            // 清空缓存区，待会用于存储子树内的所有点
            PointVector().swap(Rebuild_PCL_Storage);
            // Lock Search  禁用查找功能
            pthread_mutex_lock(&search_flag_mutex);
            // 如果正在进行查找，则等待
            while (search_mutex_counter != 0)
            {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1;
            pthread_mutex_unlock(&search_flag_mutex);
            // Lock deleted points cache 禁用删除功能
            pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);
            // 将树展平，将子树中的所有点存到Rebuild_PCL_Storage，注意输入参数中的`MULTI_THREAD_REC`是指定是否记录删除点，这些删除点估计是用来做debug
            flatten(*Rebuild_Ptr, Rebuild_PCL_Storage, MULTI_THREAD_REC);
            // Unlock deleted points cache  记录完待重建子树的点，解锁删除功能
            pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
            // Unlock Search 解锁搜索功能，将search_mutex_counter归零
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);
            pthread_mutex_unlock(&working_flag_mutex);
            /* Rebuild and update missed operations*/
            Operation_Logger_Type Operation;
            KD_TREE_NODE *new_root_node = nullptr;
            // 如果展平的树的节点大于0，则可以开始重建
            if (int(Rebuild_PCL_Storage.size()) > 0)
            {   // 重建树，树的根节点为new_root_node
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size() - 1, Rebuild_PCL_Storage, false);
                // Rebuild has been done. Updates the blocked operations into the new tree
                pthread_mutex_lock(&working_flag_mutex);
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                int tmp_counter = 0;
                // 补作业：重建树完成后，将中途的更新操作（插入、重插入、删除）操作补充
                while (!Rebuild_Logger.empty())
                {    // 从Rebuild_Logger读取操作，是插入？重插入？还是删除？
                    Operation = Rebuild_Logger.front();
                    max_queue_size = max(max_queue_size, Rebuild_Logger.size());
                    Rebuild_Logger.pop();
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    pthread_mutex_unlock(&working_flag_mutex);
                    // 根据Operation中的枚举变量指定在新的树中操作
                    run_operation(&new_root_node, Operation);
                    tmp_counter++;
                    if (tmp_counter % 10 == 0)
                        usleep(1);
                    pthread_mutex_lock(&working_flag_mutex);
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            /* Replace to original tree*/
            // pthread_mutex_lock(&working_flag_mutex); 
            pthread_mutex_lock(&search_flag_mutex); // 等待查找功能的结束
            while (search_mutex_counter != 0)
            {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1;
            pthread_mutex_unlock(&search_flag_mutex);
            // 用新树替换原来的树
            if (father_ptr->left_son_ptr == *Rebuild_Ptr)
            {
                father_ptr->left_son_ptr = new_root_node;
            }
            else if (father_ptr->right_son_ptr == *Rebuild_Ptr)
            {
                father_ptr->right_son_ptr = new_root_node;
            }
            else
            {
                throw "Error: Father ptr incompatible with current node\n";
            }
            if (new_root_node != nullptr)
                new_root_node->father_ptr = father_ptr;
            (*Rebuild_Ptr) = new_root_node;
            int valid_old = old_root_node->TreeSize - old_root_node->invalid_point_num;
            int valid_new = new_root_node->TreeSize - new_root_node->invalid_point_num;
            if (father_ptr == STATIC_ROOT_NODE)
                Root_Node = STATIC_ROOT_NODE->left_son_ptr;
            KD_TREE_NODE *update_root = *Rebuild_Ptr;
            while (update_root != nullptr && update_root != Root_Node)
            {
                update_root = update_root->father_ptr;
                if (update_root->working_flag)
                    break;
                if (update_root == update_root->father_ptr->left_son_ptr && update_root->father_ptr->need_push_down_to_left)
                    break;
                if (update_root == update_root->father_ptr->right_son_ptr && update_root->father_ptr->need_push_down_to_right)
                    break;
                Update(update_root);
            }
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);
            Rebuild_Ptr = nullptr;
            pthread_mutex_unlock(&working_flag_mutex);
            rebuild_flag = false;
            /* Delete discarded tree nodes */
            delete_tree_nodes(&old_root_node);
        }
        else
        {
            pthread_mutex_unlock(&working_flag_mutex);
        }
        pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&termination_flag_mutex_lock);
        terminated = termination_flag;
        pthread_mutex_unlock(&termination_flag_mutex_lock);
        usleep(100);
    }
    printf("Rebuild thread terminated normally\n");
}

template <typename PointType>
void KD_TREE<PointType>::run_operation(KD_TREE_NODE **root, Operation_Logger_Type operation)
{
    switch (operation.op)
    {
    case ADD_POINT:
        Add_by_point(root, operation.point, false, (*root)->division_axis);
        break;
    case ADD_BOX:
        Add_by_range(root, operation.boxpoint, false);
        break;
    case DELETE_POINT:
        Delete_by_point(root, operation.point, false);
        break;
    case DELETE_BOX:
        Delete_by_range(root, operation.boxpoint, false, false);
        break;
    case DOWNSAMPLE_DELETE:
        Delete_by_range(root, operation.boxpoint, false, true);
        break;
    case PUSH_DOWN:
        (*root)->tree_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->point_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->tree_deleted = operation.tree_deleted || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = (*root)->tree_deleted || (*root)->point_downsample_deleted;
        if (operation.tree_downsample_deleted)
            (*root)->down_del_num = (*root)->TreeSize;
        if (operation.tree_deleted)
            (*root)->invalid_point_num = (*root)->TreeSize;
        else
            (*root)->invalid_point_num = (*root)->down_del_num;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        break;
    default:
        break;
    }
}

template <typename PointType>
void KD_TREE<PointType>::Build(PointVector point_cloud)
{
    if (Root_Node != nullptr)    // 避免用户复用同一个ikd-tree对象导致错误
    {
        delete_tree_nodes(&Root_Node);
    }
    if (point_cloud.size() == 0)    // 点云的size为 0 不建树
        return;
    STATIC_ROOT_NODE = new KD_TREE_NODE;// 在堆上new一个节点作为树的根节点，不过它是一个虚节点
    InitTreeNode(STATIC_ROOT_NODE);// 初始化当前节点的属性，使用虚节点作为树的根节点

    // 注意当前左子树为空指针，为了传入函数，用了取址符号&，因此传入的是左子树指针的地址，也就是指针的指针
    // 这里使用多线程加速，设置最大线程数为10
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size() - 1, point_cloud, true, 0, 10);
    
    //更新root的属性
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    // 将虚根节点的左子节点设为树的实际根节点
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;
}

template <typename PointType>
void KD_TREE<PointType>::BuildAndSave(PointVector point_cloud,const std::string bin_pan_to_save )
{
    if (Root_Node != nullptr)    // 避免用户复用同一个ikd-tree对象导致错误
    {
        delete_tree_nodes(&Root_Node);
    }
    if (point_cloud.size() == 0)    // 点云的size为 0 不建树
        return;
    STATIC_ROOT_NODE = new KD_TREE_NODE;// 在堆上new一个节点作为树的根节点，不过它是一个虚节点
    InitTreeNode(STATIC_ROOT_NODE);// 初始化当前节点的属性，使用虚节点作为树的根节点

    // 注意当前左子树为空指针，为了传入函数，用了取址符号&，因此传入的是左子树指针的地址，也就是指针的指针
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size() - 1, point_cloud ,false);
    
    //更新root的属性
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    // 将虚根节点的左子节点设为树的实际根节点
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;
    save_ikd_tree_into_bin(bin_pan_to_save);
}

template <typename PointType>
void KD_TREE<PointType>::LoadAndBuild(const std::string bin_pan_to_load)
{
    if (Root_Node != nullptr)    // 避免用户复用同一个ikd-tree对象导致错误
    {
        delete_tree_nodes(&Root_Node);
    }
    
    Root_Node = new KD_TREE_NODE;// 在堆上new一个节点作为树的根节点，不过它是一个虚节点
    InitTreeNode(Root_Node);// 初始化节点的属性
    std::cout<<"to load_ikd_tree_from_bin"<<std::endl;
    load_ikd_tree_from_bin(bin_pan_to_load);
    //更新root的属性
    Update(Root_Node);

}

template <typename PointType>
void KD_TREE<PointType>::Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, float max_dist)
{
    MANUAL_HEAP q(2 * k_nearest);
    q.clear();
    vector<float>().swap(Point_Distance);
// #pragma omp critical 
{    
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        Search(Root_Node, k_nearest, point, q, max_dist);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        while (search_mutex_counter == -1)
        {
            pthread_mutex_unlock(&search_flag_mutex);
            usleep(1);
            pthread_mutex_lock(&search_flag_mutex);
        }
        search_mutex_counter += 1;
        pthread_mutex_unlock(&search_flag_mutex);
        Search(Root_Node, k_nearest, point, q, max_dist);
        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;
        pthread_mutex_unlock(&search_flag_mutex);
    }
}
    int k_found = min(k_nearest, int(q.size()));
    PointVector().swap(Nearest_Points);
    vector<float>().swap(Point_Distance);
    for (int i = 0; i < k_found; i++)
    {
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage)
{
    Storage.clear();
    Search_by_range(Root_Node, Box_of_Point, Storage);
}

template <typename PointType>
void KD_TREE<PointType>::Radius_Search(PointType point, const float radius, PointVector &Storage)
{
    Storage.clear();
    Search_by_radius(Root_Node, point, radius, Storage);
}

template <typename PointType>
int KD_TREE<PointType>::Add_Points(PointVector &PointToAdd, bool downsample_on)
{
    int NewPointSize = PointToAdd.size();
    int tree_size = size();
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    float min_dist, tmp_dist;
    int tmp_counter = 0;
    for (int i = 0; i < PointToAdd.size(); i++)
    {
        if (downsample_switch)
        {
            Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0] + downsample_size;
            Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1] + downsample_size;
            Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2] + downsample_size;
            mid_point.x = Box_of_Point.vertex_min[0] + (Box_of_Point.vertex_max[0] - Box_of_Point.vertex_min[0]) / 2.0;
            mid_point.y = Box_of_Point.vertex_min[1] + (Box_of_Point.vertex_max[1] - Box_of_Point.vertex_min[1]) / 2.0;
            mid_point.z = Box_of_Point.vertex_min[2] + (Box_of_Point.vertex_max[2] - Box_of_Point.vertex_min[2]) / 2.0;
            PointVector().swap(Downsample_Storage);
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);
            min_dist = calc_dist(PointToAdd[i], mid_point);
            downsample_result = PointToAdd[i];
            for (int index = 0; index < Downsample_Storage.size(); index++)
            {
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist)
                {
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
            {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result))
                {
                    if (Downsample_Storage.size() > 0)
                        Delete_by_range(&Root_Node, Box_of_Point, true, true);
                    Add_by_point(&Root_Node, downsample_result, true, Root_Node->division_axis);
                    tmp_counter++;
                }
            }
            else
            {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result))
                {
                    Operation_Logger_Type operation_delete, operation;
                    operation_delete.boxpoint = Box_of_Point;
                    operation_delete.op = DOWNSAMPLE_DELETE;
                    operation.point = downsample_result;
                    operation.op = ADD_POINT;
                    pthread_mutex_lock(&working_flag_mutex);
                    if (Downsample_Storage.size() > 0)
                        Delete_by_range(&Root_Node, Box_of_Point, false, true);
                    Add_by_point(&Root_Node, downsample_result, false, Root_Node->division_axis);
                    tmp_counter++;
                    if (rebuild_flag)
                    {
                        pthread_mutex_lock(&rebuild_logger_mutex_lock);
                        if (Downsample_Storage.size() > 0)
                            Rebuild_Logger.push(operation_delete);
                        Rebuild_Logger.push(operation);
                        pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    }
                    pthread_mutex_unlock(&working_flag_mutex);
                };
            }
        }
        else
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
            {
                Add_by_point(&Root_Node, PointToAdd[i], true, Root_Node->division_axis);
            }
            else
            {
                Operation_Logger_Type operation;
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;
                pthread_mutex_lock(&working_flag_mutex);
                Add_by_point(&Root_Node, PointToAdd[i], false, Root_Node->division_axis);
                if (rebuild_flag)
                {
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                    Rebuild_Logger.push(operation);
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&working_flag_mutex);
            }
        }
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Add_Point_Boxes(vector<BoxPointType> &BoxPoints)
{
    for (int i = 0; i < BoxPoints.size(); i++)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            Add_by_range(&Root_Node, BoxPoints[i], true);
        }
        else
        {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = ADD_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_range(&Root_Node, BoxPoints[i], false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_Points(PointVector &PointToDel)
{
    for (int i = 0; i < PointToDel.size(); i++)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            Delete_by_point(&Root_Node, PointToDel[i], true);
        }
        else
        {
            Operation_Logger_Type operation;
            operation.point = PointToDel[i];
            operation.op = DELETE_POINT;
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&Root_Node, PointToDel[i], false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

// 删除多个体素的点 关键入口函数，删除指定Box内的点
template <typename PointType>
int KD_TREE<PointType>::Delete_Point_Boxes(vector<BoxPointType> &BoxPoints)
{
    int tmp_counter = 0;
    for (int i = 0; i < BoxPoints.size(); i++)  // 遍历所有box，逐个删除
    {   // 无并行线程时，直接执行删除
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], true, false);
        }
        else
        {   // 如果此时有并行的re-balancing线程，需要通过锁(互斥量)访问
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = DELETE_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], false, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::acquire_removed_points(PointVector &removed_points)
{
    pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);
    for (int i = 0; i < Points_deleted.size(); i++)
    {
        removed_points.push_back(Points_deleted[i]);
    }
    for (int i = 0; i < Multithread_Points_deleted.size(); i++)
    {
        removed_points.push_back(Multithread_Points_deleted[i]);
    }
    Points_deleted.clear();
    Multithread_Points_deleted.clear();
    pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
    return;
}

template <typename PointType>
void KD_TREE<PointType>::BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage, bool use_multi_thread,int working_thread_num, int worker_thread_max)
{   // use_multi_thread是指定是否使用多线程，working_thread_num是当前正在工作的线程数，worker_thread_max是最大线程数
    if (l > r)
        return;
    *root = new KD_TREE_NODE;    //给**root配内存
    InitTreeNode(*root);         // 初始化根节点
    int mid = (l + r) >> 1;     // 相当于mid=(1+r)/2
    int div_axis = 0;           // 初始化划分轴
    int i;
    // Find the best division Axis 也即分布最分散的那个轴，或者说最大值减最小值之差最大的那个轴
    // 选取最优的划分轴，这里优先选用这团点云中区间（不是方差哦）最大的那个轴
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0, 0, 0};
    // 找到这团点云的包络盒
    for (i = l; i <= r; i++)
    {
        min_value[0] = min(min_value[0], Storage[i].x);
        min_value[1] = min(min_value[1], Storage[i].y);
        min_value[2] = min(min_value[2], Storage[i].z);
        max_value[0] = max(max_value[0], Storage[i].x);
        max_value[1] = max(max_value[1], Storage[i].y);
        max_value[2] = max(max_value[2], Storage[i].z);
    }
    // Select the longest dimension as division axis
    // 选用跨度最大的维度，也就是这个box的长宽高里面最大的那个轴
    for (i = 0; i < 3; i++)
        dim_range[i] = max_value[i] - min_value[i];
    for (i = 1; i < 3; i++)
        if (dim_range[i] > dim_range[div_axis])
            div_axis = i;
    // Divide by the division axis and recursively build.

    (*root)->division_axis = div_axis;
    /*********************************************
     * nth_element(first, nth, last, compare)
     求[first, last]这个区间中第n大小的元素，如果参数加入了compare函数，就按compare函数的方式比较。
     这一段就是找切割维度的中位数

     nth_element(a,a+k,a+n)，函数只是把下标为k的元素放在了正确位置，对其它元素并没有排序，
     当然k左边元素都小于等于它，右边元素都大于等于它，所以可以利用这个函数快速定位某个元素。
    ******************************************************/
    switch (div_axis)
    {
    case 0:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        break;
    case 1:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_y);
        break;
    case 2:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_z);
        break;
    default:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        break;
    }
    (*root)->point = Storage[mid];
    KD_TREE_NODE *left_son = nullptr, *right_son = nullptr;

    
    if(use_multi_thread&&( working_thread_num <= worker_thread_max)) // 
    {   
        working_thread_num++;
        auto thread1 = std::thread(&KD_TREE<PointType>::BuildTree, this, &left_son, l, mid - 1, std::ref(Storage),
                                     true, working_thread_num, worker_thread_max);

        auto thread2 = std::thread(&KD_TREE<PointType>::BuildTree, this, &right_son, mid + 1, r, std::ref(Storage),
                                     true, working_thread_num, worker_thread_max);
        working_thread_num++;
        thread1.join();
        thread2.join();
    }
    else
    {
        BuildTree(&left_son, l, mid - 1, Storage, false, working_thread_num, worker_thread_max);
        BuildTree(&right_son, mid + 1, r, Storage, false, working_thread_num, worker_thread_max);
    }
    
    // if(depth < worker_depth){
    //     auto thread1 = std::thread(&KD_TREE<PointType>::BuildTree, this, &left_son, l, mid - 1, std::ref(Storage), depth + 1, worker_depth);
    //     auto thread2 = std::thread(&KD_TREE<PointType>::BuildTree, this, &right_son, mid + 1, r, std::ref(Storage), depth + 1, worker_depth);
    //     thread1.join();
    //     thread2.join();
    // } else {
    //     BuildTree(&left_son, l, mid - 1, Storage, depth + 1);
    //     BuildTree(&right_son, mid + 1, r, Storage, depth + 1);
    // }

    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;
    Update((*root));
    return;
}


template <typename PointType>
void KD_TREE<PointType>::save_ikd_tree_into_bin(const std::string bin_path )
{
    // std::ofstream out(bin_path.c_str(), std::ios::binary);
    std::ofstream out(bin_path.c_str(), std::ios::out | std::ios::binary);

    Serialize(Root_Node, out);
    out.close();
}

template <typename PointType>
void KD_TREE<PointType>::Serialize(KD_TREE_NODE *node, std::ofstream &out) 
{   
    /*
    编码设置：
    1. bool empty_or_not_sym：是否为空节点，如果为空节点，后面的数据都不用写入，这样读如的时候可以直接判断是否为空节点
    2. PointType point：点云内容
    3. int division_axis：划分轴
    4. int left_son_ptr：左子树指针
    5. int right_son_ptr：右子树指针
    6. bool is_leaf：是否为叶子节点
    7. bool is_sym：是否为对称节点
    8. bool is_empty：是否为空节点
    9. float min_x：包围盒最小值
    10. float min_y：包围盒最小值
    11. float min_z：包围盒最小值
    12. float max_x：包围盒最大值
    13. float max_y：包围盒最大值
    14. float max_z：包围盒最大值
    15. float center_x：包围盒中心
    16. float center_y：包围盒中心
    17. float center_z：包围盒中心
    */
    if (node == nullptr) 
    {
        char *buffer = new char[ sizeof(bool) ];
        bool empty_or_not_sym = true;
        std::memcpy(buffer + 0, &(empty_or_not_sym), sizeof(bool));

        out.write((char*)&buffer, sizeof(bool) );

        delete[] buffer;
        // std::cout<<"node == nullptr\n";
        return;
    }

    char *buffer = new char[ sizeof(bool) + sizeof(PointType) + 4 * sizeof(int) + 7 * sizeof(bool) + 9 * sizeof(float)];

    bool empty_or_not_sym = false;
    std::memcpy(buffer + 0, &(empty_or_not_sym), sizeof(bool));


    std::memcpy(buffer + 0, &(node->point ), sizeof(PointType));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType), &(node->division_axis), sizeof(int));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType)+ 1 * sizeof(int), &(node->TreeSize), sizeof(int));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType)+ 2 * sizeof(int), &(node->invalid_point_num), sizeof(int));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 3 * sizeof(int), &(node->down_del_num), sizeof(int));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int), &(node->point_deleted), sizeof(bool));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + sizeof(bool), &(node->tree_deleted), sizeof(bool));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 2 * sizeof(bool), &(node->point_downsample_deleted), sizeof(bool));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 3 * sizeof(bool), &(node->tree_downsample_deleted), sizeof(bool));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 4 * sizeof(bool), &(node->need_push_down_to_left), sizeof(bool));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 5 * sizeof(bool), &(node->need_push_down_to_right), sizeof(bool));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 6 * sizeof(bool), &(node->working_flag), sizeof(bool));
    // std::memcpy(buffer + sizeof(PointType) + 4 * sizeof(int) + 7 * sizeof(bool), &(node->push_down_mutex_lock), sizeof(pthread_mutex_t));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 7 * sizeof(bool) , &(node->node_range_x), 2 * sizeof(float));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 7 * sizeof(bool) + 2 * sizeof(float), &(node->node_range_y), 2 * sizeof(float));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 7 * sizeof(bool) + 4 * sizeof(float), &(node->node_range_z), 2 * sizeof(float));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 7 * sizeof(bool) + 6 * sizeof(float), &(node->radius_sq), sizeof(float));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 7 * sizeof(bool) + 7 * sizeof(float), &(node->alpha_del), sizeof(float));
    std::memcpy(buffer + sizeof(bool)+ sizeof(PointType) + 4 * sizeof(int) + 7 * sizeof(bool) + 8 * sizeof(float), &(node->alpha_bal), sizeof(float));

    
    out.write((char*)&buffer, sizeof(PointType) + 4 * sizeof(int) + 8 * sizeof(bool) + 9 * sizeof(float));
    
    delete[] buffer;

    Serialize(node->left_son_ptr, out);
    Serialize(node->right_son_ptr, out);
}

template <typename PointType>
typename KD_TREE<PointType>::KD_TREE_NODE* Deserialize(std::ifstream &in);
template <typename PointType>
typename KD_TREE<PointType>::KD_TREE_NODE* KD_TREE<PointType>::Deserialize(std::ifstream &in) // 反序列化
{   
    if (!in.good()) 
    {
        std::cerr << "Error: Could not open " << std::endl;
        return nullptr;
    }
    std::cout << "11" << std::endl;
    
    
    bool empty_or_not_node;
    in.read((char*)&empty_or_not_node, sizeof(bool));
    std::cout<<"empty_or_not_node: "<<empty_or_not_node<<std::endl;
    // char *buffer = new char[ sizeof(bool) ];
    // in.read(buffer, sizeof(bool));
    // std::memcpy(&empty_or_not_node, buffer, sizeof(bool));
    if (!in) // Check for read error
    {
        std::cerr << "Error: Failed to read from file" << std::endl;
        return nullptr;
    }
    std::cout << "12" << std::endl;

    if (empty_or_not_node) 
    {
        // delete[] buffer;
        return nullptr;
    }

    // delete[] buffer;
    std::cout << "13" << std::endl;


    KD_TREE_NODE *node = new KD_TREE_NODE;
    InitTreeNode(node) ;// 这里的初始化是为了防止反序列化的时候，没有初始化，导致内存泄漏
        std::cout << "14" << std::endl;

    in.read(reinterpret_cast<char*>(&(node->point)), sizeof(PointType));
    in.read(reinterpret_cast<char*>(&(node->division_axis)), sizeof(int));
    in.read(reinterpret_cast<char*>(&(node->TreeSize)), sizeof(int));
    in.read(reinterpret_cast<char*>(&(node->invalid_point_num)), sizeof(int));
    in.read(reinterpret_cast<char*>(&(node->down_del_num)), sizeof(int));
    in.read(reinterpret_cast<char*>(&(node->point_deleted)), sizeof(bool));
    in.read(reinterpret_cast<char*>(&(node->tree_deleted)), sizeof(bool));
    in.read(reinterpret_cast<char*>(&(node->point_downsample_deleted)), sizeof(bool));
    in.read(reinterpret_cast<char*>(&(node->tree_downsample_deleted)), sizeof(bool));
    in.read(reinterpret_cast<char*>(&(node->need_push_down_to_left)), sizeof(bool));
    in.read(reinterpret_cast<char*>(&(node->need_push_down_to_right)), sizeof(bool));
    in.read(reinterpret_cast<char*>(&(node->working_flag)), sizeof(bool));
    in.read(reinterpret_cast<char*>(&(node->node_range_x)), 2 * sizeof(float));
    in.read(reinterpret_cast<char*>(&(node->node_range_y)), 2 * sizeof(float));
    in.read(reinterpret_cast<char*>(&(node->node_range_z)), 2 * sizeof(float));
    in.read(reinterpret_cast<char*>(&(node->radius_sq)), sizeof(float));
    in.read(reinterpret_cast<char*>(&(node->alpha_del)), sizeof(float));
    in.read(reinterpret_cast<char*>(&(node->alpha_bal)), sizeof(float));



    // PointType point;
    // in.read((char*)&point, sizeof(PointType));
    // if (point.x == -1 && point.y == -1 && point.z == -1) 
    // {
    //     return nullptr; // 空节点
    // }
    // KD_TREE_NODE *node = new KD_TREE_NODE;
    // node->point = point;

    std::cout << "22" << std::endl;


    node->left_son_ptr = Deserialize(in);
    node->right_son_ptr = Deserialize(in);
    
    Update(node);
    return node;
}

template <typename PointType>
void KD_TREE<PointType>::load_ikd_tree_from_bin(const std::string bin_path)
{
    std::ifstream in(bin_path.c_str(),  std::ios::in | std::ios::binary);
    if (!in.good())
    {
        std::cerr << "Failed to open file: " << bin_path << std::endl;
        return;
    }
    std::cout << "to Deserialize" << std::endl;
    Root_Node = Deserialize(in);
    in.close();
}

template <typename PointType>
void KD_TREE<PointType>::Rebuild(KD_TREE_NODE **root)
{
    KD_TREE_NODE *father_ptr;
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num)
    {
        if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock))
        {
            if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize))
            {
                Rebuild_Ptr = root;
            }
            pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        }
    }
    else
    {
        father_ptr = (*root)->father_ptr;
        int size_rec = (*root)->TreeSize;
        PCL_Storage.clear();
        flatten(*root, PCL_Storage, DELETE_POINTS_REC);
        delete_tree_nodes(root);
        BuildTree(root, 0, PCL_Storage.size() - 1, PCL_Storage, 0);
        if (*root != nullptr)
            (*root)->father_ptr = father_ptr;
        if (*root == Root_Node)
            STATIC_ROOT_NODE->left_son_ptr = *root;
    }
    return;
}

template <typename PointType>
int KD_TREE<PointType>::Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample)
{
    if ((*root) == nullptr || (*root)->tree_deleted)
        return 0;
    (*root)->working_flag = true;
    Push_Down(*root);
    int tmp_counter = 0;
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1])
        return 0;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1])
        return 0;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1])
        return 0;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        tmp_counter = (*root)->TreeSize - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->TreeSize;
        if (is_downsample)
        {
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->TreeSize;
        }
        return tmp_counter;
    }
    if (!(*root)->point_deleted && boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = true;
        tmp_counter += 1;
        if (is_downsample)
            (*root)->point_downsample_deleted = true;
    }
    Operation_Logger_Type delete_box_log;
    struct timespec Timeout;
    if (is_downsample)
        delete_box_log.op = DOWNSAMPLE_DELETE;
    else
        delete_box_log.op = DELETE_BOX;
    delete_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
    {
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
    {
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild)
{
    if ((*root) == nullptr || (*root)->tree_deleted)
        return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (same_point((*root)->point, point) && !(*root)->point_deleted)
    {
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize)
            (*root)->tree_deleted = true;
        return;
    }
    Operation_Logger_Type delete_log;
    struct timespec Timeout;
    delete_log.op = DELETE_POINT;
    delete_log.point = point;
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z))
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
        {
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->left_son_ptr, point, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    else
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
        {
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->right_son_ptr, point, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild)
{
    if ((*root) == nullptr)
        return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num;
        return;
    }
    if (boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }
    Operation_Logger_Type add_box_log;
    struct timespec Timeout;
    add_box_log.op = ADD_BOX;
    add_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
    {
        Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->left_son_ptr), boxpoint, false);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
    {
        Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->right_son_ptr), boxpoint, false);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild, int father_axis)
{
    if (*root == nullptr)
    {
        *root = new KD_TREE_NODE;
        InitTreeNode(*root);
        (*root)->point = point;
        (*root)->division_axis = (father_axis + 1) % 3;
        Update(*root);
        return;
    }
    (*root)->working_flag = true;
    Operation_Logger_Type add_log;
    struct timespec Timeout;
    add_log.op = ADD_POINT;
    add_log.point = point;
    Push_Down(*root);
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z))
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
        {
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild, (*root)->division_axis);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->left_son_ptr, point, false, (*root)->division_axis);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    else
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
        {
            Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild, (*root)->division_axis);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->right_son_ptr, point, false, (*root)->division_axis);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, float max_dist)
{
    if (root == nullptr || root->tree_deleted)
        return;
    float cur_dist = calc_box_dist(root, point);
    float max_dist_sqr = max_dist * max_dist;
    if (cur_dist > max_dist_sqr)
        return;
    int retval;
    if (root->need_push_down_to_left || root->need_push_down_to_right)
    {
        retval = pthread_mutex_trylock(&(root->push_down_mutex_lock));
        if (retval == 0)
        {
            Push_Down(root);
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
        else
        {
            pthread_mutex_lock(&(root->push_down_mutex_lock));
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
    }
    if (!root->point_deleted)
    {
        float dist = calc_dist(point, root->point);
        if (dist <= max_dist_sqr && (q.size() < k_nearest || dist < q.top().dist))
        {
            if (q.size() >= k_nearest)
                q.pop();
            PointType_CMP current_point{root->point, dist};
            q.push(current_point);
        }
    }
    int cur_search_counter;
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    if (q.size() < k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist)
    {
        if (dist_left_node <= dist_right_node)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
            {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist)
            {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
                {
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                }
                else
                {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
        else
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
            {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist)
            {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
                {
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                }
                else
                {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
    }
    else
    {
        if (dist_left_node < q.top().dist)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
            {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
        if (dist_right_node < q.top().dist)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
            {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    if (boxpoint.vertex_max[0] <= root->node_range_x[0] || boxpoint.vertex_min[0] > root->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] || boxpoint.vertex_min[1] > root->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= root->node_range_z[0] || boxpoint.vertex_min[2] > root->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] && boxpoint.vertex_max[0] > root->node_range_x[1] && boxpoint.vertex_min[1] <= root->node_range_y[0] && boxpoint.vertex_max[1] > root->node_range_y[1] && boxpoint.vertex_min[2] <= root->node_range_z[0] && boxpoint.vertex_max[2] > root->node_range_z[1])
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (boxpoint.vertex_min[0] <= root->point.x && boxpoint.vertex_max[0] > root->point.x && boxpoint.vertex_min[1] <= root->point.y && boxpoint.vertex_max[1] > root->point.y && boxpoint.vertex_min[2] <= root->point.z && boxpoint.vertex_max[2] > root->point.z)
    {
        if (!root->point_deleted)
            Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr)
    {
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr)
    {
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    PointType range_center;
    range_center.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
    range_center.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;
    range_center.z = (root->node_range_z[0] + root->node_range_z[1]) * 0.5;
    float dist = sqrt(calc_dist(range_center, point));
    if (dist > radius + sqrt(root->radius_sq)) return;
    if (dist <= radius - sqrt(root->radius_sq)) 
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (!root->point_deleted && calc_dist(root->point, point) <= radius * radius){
        Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr)
    {
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr)
    {
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }    
    return;
}

template <typename PointType>
bool KD_TREE<PointType>::Criterion_Check(KD_TREE_NODE *root)
{
    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size)
    {
        return false;
    }
    float balance_evaluation = 0.0f;
    float delete_evaluation = 0.0f;
    KD_TREE_NODE *son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr)
        son_ptr = root->right_son_ptr;
    delete_evaluation = float(root->invalid_point_num) / root->TreeSize;
    balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize - 1);
    if (delete_evaluation > delete_criterion_param)
    {
        return true;
    }
    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1 - balance_criterion_param)
    {
        return true;
    }
    return false;
}

template <typename PointType>
void KD_TREE<PointType>::Push_Down(KD_TREE_NODE *root)
{
    if (root == nullptr)
        return;
    Operation_Logger_Type operation;
    operation.op = PUSH_DOWN;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;
    if (root->need_push_down_to_left && root->left_son_ptr != nullptr)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
        {
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
            else
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_left = false;
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
            else
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_left = false;
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    if (root->need_push_down_to_right && root->right_son_ptr != nullptr)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
        {
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
            else
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_right = false;
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
            else
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_right = false;
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Update(KD_TREE_NODE *root)
{
    KD_TREE_NODE *left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE *right_son_ptr = root->right_son_ptr;
    float tmp_range_x[2] = {INFINITY, -INFINITY};
    float tmp_range_y[2] = {INFINITY, -INFINITY};
    float tmp_range_z[2] = {INFINITY, -INFINITY};
    // Update Tree Size
    if (left_son_ptr != nullptr && right_son_ptr != nullptr)
    {
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num + right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !right_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(min(left_son_ptr->node_range_x[0], right_son_ptr->node_range_x[0]), root->point.x);
            tmp_range_x[1] = max(max(left_son_ptr->node_range_x[1], right_son_ptr->node_range_x[1]), root->point.x);
            tmp_range_y[0] = min(min(left_son_ptr->node_range_y[0], right_son_ptr->node_range_y[0]), root->point.y);
            tmp_range_y[1] = max(max(left_son_ptr->node_range_y[1], right_son_ptr->node_range_y[1]), root->point.y);
            tmp_range_z[0] = min(min(left_son_ptr->node_range_z[0], right_son_ptr->node_range_z[0]), root->point.z);
            tmp_range_z[1] = max(max(left_son_ptr->node_range_z[1], right_son_ptr->node_range_z[1]), root->point.z);
        }
        else
        {
            if (!left_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!right_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (left_son_ptr != nullptr)
    {
        root->TreeSize = left_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(left_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] = max(left_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] = min(left_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] = max(left_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] = min(left_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] = max(left_son_ptr->node_range_z[1], root->point.z);
        }
        else
        {
            if (!left_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (right_son_ptr != nullptr)
    {
        root->TreeSize = right_son_ptr->TreeSize + 1;
        root->invalid_point_num = right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!right_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(right_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] = max(right_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] = min(right_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] = max(right_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] = min(right_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] = max(right_son_ptr->node_range_z[1], root->point.z);
        }
        else
        {
            if (!right_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else
    {
        root->TreeSize = 1;
        root->invalid_point_num = (root->point_deleted ? 1 : 0);
        root->down_del_num = (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = root->point_downsample_deleted;
        root->tree_deleted = root->point_deleted;
        tmp_range_x[0] = root->point.x;
        tmp_range_x[1] = root->point.x;
        tmp_range_y[0] = root->point.y;
        tmp_range_y[1] = root->point.y;
        tmp_range_z[0] = root->point.z;
        tmp_range_z[1] = root->point.z;
    }
    memcpy(root->node_range_x, tmp_range_x, sizeof(tmp_range_x));
    memcpy(root->node_range_y, tmp_range_y, sizeof(tmp_range_y));
    memcpy(root->node_range_z, tmp_range_z, sizeof(tmp_range_z));
    float x_L = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
    float y_L = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;
    float z_L = (root->node_range_z[1] - root->node_range_z[0]) * 0.5;
    root->radius_sq = x_L*x_L + y_L * y_L + z_L * z_L;
    if (left_son_ptr != nullptr)
        left_son_ptr->father_ptr = root;
    if (right_son_ptr != nullptr)
        right_son_ptr->father_ptr = root;
    if (root == Root_Node && root->TreeSize > 3)
    {
        KD_TREE_NODE *son_ptr = root->left_son_ptr;
        if (son_ptr == nullptr)
            son_ptr = root->right_son_ptr;
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize - 1);
        root->alpha_del = float(root->invalid_point_num) / root->TreeSize;
        root->alpha_bal = (tmp_bal >= 0.5 - EPSS) ? tmp_bal : 1 - tmp_bal;
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    if (!root->point_deleted)
    {
        Storage.push_back(root->point);
    }
    flatten(root->left_son_ptr, Storage, storage_type);
    flatten(root->right_son_ptr, Storage, storage_type);
    switch (storage_type)
    {
    case NOT_RECORD:
        break;
    case DELETE_POINTS_REC:
        if (root->point_deleted && !root->point_downsample_deleted)
        {
            Points_deleted.push_back(root->point);
        }
        break;
    case MULTI_THREAD_REC:
        if (root->point_deleted && !root->point_downsample_deleted)
        {
            Multithread_Points_deleted.push_back(root->point);
        }
        break;
    default:
        break;
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::delete_tree_nodes(KD_TREE_NODE **root)
{
    if (*root == nullptr)
        return;
    Push_Down(*root);
    delete_tree_nodes(&(*root)->left_son_ptr);
    delete_tree_nodes(&(*root)->right_son_ptr);

    pthread_mutex_destroy(&(*root)->push_down_mutex_lock);
    delete *root;
    *root = nullptr;

    return;
}

template <typename PointType>
bool KD_TREE<PointType>::same_point(PointType a, PointType b)
{
    return (fabs(a.x - b.x) < EPSS && fabs(a.y - b.y) < EPSS && fabs(a.z - b.z) < EPSS);
}

template <typename PointType>
float KD_TREE<PointType>::calc_dist(PointType a, PointType b)
{
    float dist = 0.0f;
    dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    return dist;
}

template <typename PointType>
float KD_TREE<PointType>::calc_box_dist(KD_TREE_NODE *node, PointType point)
{
    if (node == nullptr)
        return INFINITY;
    float min_dist = 0.0;
    if (point.x < node->node_range_x[0])
        min_dist += (point.x - node->node_range_x[0]) * (point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1])
        min_dist += (point.x - node->node_range_x[1]) * (point.x - node->node_range_x[1]);
    if (point.y < node->node_range_y[0])
        min_dist += (point.y - node->node_range_y[0]) * (point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1])
        min_dist += (point.y - node->node_range_y[1]) * (point.y - node->node_range_y[1]);
    if (point.z < node->node_range_z[0])
        min_dist += (point.z - node->node_range_z[0]) * (point.z - node->node_range_z[0]);
    if (point.z > node->node_range_z[1])
        min_dist += (point.z - node->node_range_z[1]) * (point.z - node->node_range_z[1]);
    return min_dist;
}
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_x(PointType a, PointType b) { return a.x < b.x; }
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_y(PointType a, PointType b) { return a.y < b.y; }
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_z(PointType a, PointType b) { return a.z < b.z; }

// Manual heap



// manual queue


// Manual Instatiations
template class KD_TREE<pcl::PointXYZ>;
template class KD_TREE<pcl::PointXYZI>;
template class KD_TREE<pcl::PointXYZINormal>;

