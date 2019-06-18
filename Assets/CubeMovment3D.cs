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

	public class CubeMovment3D : TrueSyncBehaviour
    {
        private TSVector mouseInputWordPos;
        public bool isScene3 = false;
        private string message = "";
  
    public override void OnSyncedUpdate()
    {
        TSVector screenPos = Camera.main.WorldToScreenPoint(tsTransform.position.ToVector()).ToTSVector();
        TSVector mouseScreenPos = Input.mousePosition.ToTSVector();
        TSVector mouseScreenVector = new TSVector(mouseScreenPos.x, mouseScreenPos.y, screenPos.z);
        mouseInputWordPos = Camera.main.ScreenToWorldPoint(mouseScreenVector.ToVector()).ToTSVector();
        tsTransform.position=new TSVector(mouseInputWordPos.x,0.74,mouseInputWordPos.z);
    }
    public void OnSyncedCollisionEnter(TSCollision other)
    {
        if(!isScene3)return;
        Debug.Log(other.gameObject.name+"+碰撞");
        message = other.gameObject.name + "+碰撞";
    }

    void OnGUI()
    {
       
       
        if (isScene3)
        {
            GUI.skin.label.normal.textColor = Color.blue;
            GUI.Label(new Rect(500, 20, 500, 50), "(3)这是一个测试TrueSync物理碰撞检测的示例，移动鼠标控制方块");
            GUI.Label(new Rect(500, 50, 500, 50), message);
        }
        else
        {
            GUI.skin.label.normal.textColor = Color.blue;
            GUI.Label(new Rect(500, 20, 500, 50), "(2)这是一个测试TrueSync物理碰撞的示例，移动鼠标控制方块");
        }

        //GUI.Label(new Rect(500,60,500,50), "(2)这是一个测试TrueSync物理碰撞的示例，移动鼠标控制方块");
        //GUI.Label(new Rect(500,80,500,50), "(2)这是一个测试TrueSync物理碰撞的示例，移动鼠标控制方块");
    }

    }
