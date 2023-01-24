
#ifndef VRP_LABELLIST_H
#define VRP_LABELLIST_H

#include "scip/scip.h"
#include "label2.h"
#include "model_data.h"
#include "vector"
#include <cstdlib>

class LabelNode {
public:
    Label2*     label2_;
    SCIP_Real   value_;
    LabelNode*  next_;
    LabelNode*  prev_;
    LabelNode*  child_;
    LabelNode*  parent_;
    LabelNode*  nextSib_;
    LabelNode*  prevSib_;
    SCIP_Bool   is_propagated_;
    LabelNode(
        Label2*     label2,
        SCIP_Real   value
    ):
    label2_(label2),
    value_(value)
    {
        next_ = nullptr;
        prev_ = nullptr;
        child_ = nullptr;
        parent_ = nullptr;
        nextSib_ = nullptr;
        prevSib_ = nullptr;
        is_propagated_ = FALSE;
    }
    ~LabelNode()
    {
        delete label2_;
    }
    /* delete node from the family tree -> adjust pointers */
    void delete_from_family_tree() const
    {
        if(prevSib_ == nullptr) /* if node is first child */
        {
            if(nextSib_ != nullptr)
                nextSib_->prevSib_ = nullptr;
            parent_->child_ = nextSib_;
        }else
        {
            prevSib_->nextSib_ = nextSib_;
            if(nextSib_ != nullptr)
                nextSib_->prevSib_ = prevSib_;
        }
    }
};

class LabelList {
public:
    LabelNode*  head_;
    int         length_;
    LabelList()
    {
        head_ = nullptr;
        length_ = 0;
    }

    ~LabelList()
    {
        LabelNode* node;
        if(length_ > 0)
        {
            while(length_ > 0)
            {
                node = extract_first();
                delete node;
            }
        }
    }

    /** inserts a LabelNode object to the LabelList */
    void insert_node(
        LabelNode*  node
    ){
        length_++;
        assert(node->next_ == nullptr);
        assert(node->prev_ == nullptr);
        /* if list is empty */
        if(head_ == nullptr)
        {
            head_ = node;
            return;
        }
        /* insertion at the start */
        if(head_->value_ >= node->value_)
        {
            node->next_ = head_;
            head_->prev_ = node;
            head_ = node;
            return;
        }
        /* insertion in the middle */
        LabelNode* tmp = head_;
        int cnt = 1;
        while (tmp->next_ != nullptr)
        {
            assert(tmp->next_ != node);
            if(tmp->next_->value_ < node->value_)
            {
                tmp = tmp->next_;
                cnt++;
            }else
            {
                node->next_ = tmp->next_;
                node->prev_ = tmp;
                tmp->next_->prev_ = node;
                tmp->next_ = node;
                return;
            }
        }
        /* insertion at the end */
        node->prev_ = tmp;
        tmp->next_ = node;
    };
    LabelNode* extract_first()
    {
        assert(length_ > 0);
        length_--;

        LabelNode* node;

        node = head_;
        head_ = head_->next_;
        if(head_ != nullptr)
            head_->prev_ = nullptr;

        return node;
    }

    void delete_from_labelList(
            LabelNode* node
    ){
        /* case that node is the head of the list */
        if(node->prev_ == nullptr)
        {
            assert(head_ == node);
            if(node->next_ != nullptr)
                node->next_->prev_ = nullptr;
            head_ = node->next_;
        }else /* node is not head of the list */
        {
            assert(head_ != node);
            assert(node->prev_ != node);
            assert(node->next_ != node);
            assert(node->prev_ != nullptr);
            node->prev_->next_ = node->next_;
            if(node->next_ != nullptr)
            {
                node->next_->prev_ = node->prev_;
                assert(node->prev_->next_ != node);
                assert(node->next_->prev_ != node);
            }
        }
        length_--;
    }
    void delete_node(
        LabelNode* node
    ){
        node->delete_from_family_tree();
        delete_from_labelList(node);
    }
};

int deleteChildren(
    std::vector<LabelList*>&    activeLists,
    std::vector<LabelList*>&    oldLists,
    LabelNode*                  node
);

int dominance_check2(
        std::vector<LabelList*>&    activeLists,
        std::vector<LabelList*>&    oldLists,
        Label2*                     label,
        SCIP_Bool                   usedLabels,
        ObjPricerVRP*               pricerData,
        SCIP*                       scip
);

#endif //VRP_LABELLIST_H
