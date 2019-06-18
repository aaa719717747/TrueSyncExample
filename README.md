# TrueSyncExample

这是一个帧同步的插件，具有确定性物理引擎，定点函数库，碰撞检测，高度封装，跟unity函数库保持高度一致。

一：其中有5个场景示例文件:

001:测试自然物理刚体学 002：测试自然物理碰撞 003：测试物理碰撞检测 004：测试复杂环境之下的物理碰撞 005：测试不规则碰撞及检测

二：检测碰撞相关消息： public void OnSyncedCollisionEnter(TSCollision other) {

    }
    public void OnSyncedCollisionStay(TSCollision other)
    {

    }
    public void OnSyncedCollisionExit(TSCollision other)
    {

    }
    public void OnSyncedTriggerEnter(TSCollision other)
    {

    }
    public void OnSyncedTriggerStay(TSCollision other)
    {

    }
    public void OnSyncedTriggerExit(TSCollision other)
    {

    }
三：注意事项： 1.0 场景中需要TrueSyncManager环境 2.0 生成的预制体需要由TrueSyncManager生成，具有TrueSyncBehavir环境 3.0 物理出现平滑位移应该指定TsMaterial控制阻尼系数，恢复系数。
