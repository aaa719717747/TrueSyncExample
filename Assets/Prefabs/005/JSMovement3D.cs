/***
*   Company:Frozen Wolf Sudio
*	Title："LockstepFrame" Lockstep框架项目
*		主题：XXX
*	Description：
*		功能：XXX
*	Date：2019
*	Version：0.1版本
*	Author：xxx
*	Modify Recoder：
*/
using System.Collections;
using System.Collections.Generic;
using TrueSync;
using UnityEngine;
namespace FightClub
{
	public class JSMovement3D : TrueSyncBehaviour
    {
        private Animator animator;
        public const byte HORIZONTAL = 0;
        public const byte VERTICAL = 1;
        private TSVector mouseInputWordPos;
        public override void OnSyncedStart()
        {
            animator = GetComponent<Animator>();
        }

        public override void OnSyncedUpdate()
        {
            //FP x = TrueSyncInput.GetFP(HORIZONTAL);
            //FP z = TrueSyncInput.GetFP(VERTICAL);
            //if (x!=0||z!=0)
            //{
            //    animator.SetInteger("JSController", 1);
            //    if (x!=0)
            //    {
            //        tsRigidBody.velocity = new TSVector(x, tsTransform.position.y, tsTransform.position.z);
            //    }
            //    if (z != 0)
            //    {
            //        tsRigidBody.velocity = new TSVector(tsTransform.position.x, tsTransform.position.y, z);
            //    }

            //}
            //else
            //{
            //    animator.SetInteger("JSController", 0);
            //}
            TSVector screenPos = Camera.main.WorldToScreenPoint(tsTransform.position.ToVector()).ToTSVector();
            TSVector mouseScreenPos = Input.mousePosition.ToTSVector();
            TSVector mouseScreenVector = new TSVector(mouseScreenPos.x, mouseScreenPos.y, screenPos.z);
            mouseInputWordPos = Camera.main.ScreenToWorldPoint(mouseScreenVector.ToVector()).ToTSVector();
            tsTransform.position = new TSVector(mouseInputWordPos.x, 0.74, mouseInputWordPos.z);
            
        }

        public override void OnSyncedInput()
        {
            FP xAxis = Input.GetAxis("Horizontal");
            FP zAxis = Input.GetAxis("Vertical");
            TrueSyncInput.SetFP(HORIZONTAL,xAxis);
            TrueSyncInput.SetFP(VERTICAL,zAxis);
        }

        public void OnSyncedCollisionEnter(TSCollision other)
        {

        }
    }

}
