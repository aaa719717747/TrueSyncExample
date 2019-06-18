using System;
using TrueSync;

public interface IPhysicsManagerBase
{
	void Init();

	void UpdateStep();

	IWorld GetWorld();

	IWorldClone GetWorldClone();

	void RemoveBody(IBody iBody);
}
