using System.Collections.Generic;

namespace TrueSync.Physics2D {

    internal class ContactEdgeClone2D {

        public ContactCloneKey contactKey;

        public Body body;

        public ContactEdgeCloneKey contactEdgeCloneKey = new ContactEdgeCloneKey();

        public ContactEdgeCloneKey nextEdge = new ContactEdgeCloneKey();

        public ContactEdgeCloneKey previousEdge = new ContactEdgeCloneKey();

        public void Clone(ContactEdge contactEdge) {
            this.contactKey = contactEdge.Contact.Key;
            this.body = contactEdge.Other;

            this.contactEdgeCloneKey.Set(contactKey, body.BodyId);

            if (contactEdge.Next != null) {
                this.nextEdge.Set(contactEdge.Next.Contact.Key, contactEdge.Next.Other.BodyId);
            } else {
                this.nextEdge.Set(this.nextEdge.contactKey, -1);
            }

            if (contactEdge.Prev != null) {
                this.previousEdge.Set(contactEdge.Prev.Contact.Key, contactEdge.Prev.Other.BodyId);
            } else {
                this.previousEdge.Set(this.previousEdge.contactKey, -1);
            }
        }

        public ContactEdge Restore(bool restoreLinks, Dictionary<ContactCloneKey, Contact> contactDic, Dictionary<ContactEdgeCloneKey, ContactEdge> contactEdgeDic) {
            if (restoreLinks) {
                ContactEdge contEdge = contactEdgeDic[contactEdgeCloneKey];

                if (nextEdge.bodyId != -1) {
                    contEdge.Next = contactEdgeDic[nextEdge];
                }

                if (previousEdge.bodyId  != -1) {
                    contEdge.Prev = contactEdgeDic[previousEdge];
                }

                return contEdge;
            } else {
                if (contactEdgeDic.ContainsKey(contactEdgeCloneKey)) {
                    return contactEdgeDic[contactEdgeCloneKey];
                }

                ContactEdge contEdge = WorldClone2D.poolContactEdge.GetNew();
                contEdge.Contact = contactDic[contactKey];
                contEdge.Other = body;

                contactEdgeDic[contactEdgeCloneKey] = contEdge;

                return contEdge;
            }
        }

    }

}