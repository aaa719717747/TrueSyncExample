using TrueSync;
using UnityEngine;

/**
* @brief Interface to a world physics manager.
**/
public interface IPhysicsManager : IPhysicsManagerBase {

    TSVector Gravity { get; set; }

    bool SpeculativeContacts { get; set; }

    FP LockedTimeStep { get;  set; }

    /**
    * @brief Returns the GameObject related to a {@link IBody}.
    **/
    GameObject GetGameObject(IBody rigidBody);

    /**
    * @brief Returns the layer related to a {@link IBody}.
    **/
    int GetBodyLayer(IBody rigidBody);

    /**
    * @brief Returns true if rigidBody1 and rigidBody2 can collide.
    **/
    bool IsCollisionEnabled(IBody rigidBody1, IBody rigidBody2);

    void AddBody(ICollider iCollider);

    void OnRemoveBody(System.Action<IBody> OnRemoveBody);

}