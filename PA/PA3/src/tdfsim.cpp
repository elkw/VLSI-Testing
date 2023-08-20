/**********************************************************************/
/*  Parallel-Fault Event-Driven Transition Delay Fault Simulator      */
/*                                                                    */
/*           Author: Tsai-Chieh Chen                                  */
/*           last update : 10/22/2018                                 */
/**********************************************************************/

#include "atpg.h"

/* pack 16 faults into one packet.  simulate 16 faults together.
 * the following variable name is somewhat misleading */
#define num_of_pattern 16

/* The faulty_wire contains a list of wires that
 * change their values in the fault simulation for a particular packet.
 * (that is wire_value1 != wire_value2)
 * Note that the wire themselves are not necessarily a fault site.
 * The list is linked by the pnext pointers */

/* fault simulate a set of test vectors */
void ATPG::transition_delay_fault_simulation(int &total_detect_num)
{
    int cnt_detected_num = 0;
    for (int i = vectors.size() - 1; i >= 0; i--)
    {
        tdf_sim_vector_one(vectors[i], cnt_detected_num);
        total_detect_num = total_detect_num + cnt_detected_num;
    }
}

void ATPG::tdf_sim_vector_one(const string &v, int &num_of_current_detect)
{
    fptr f;
    // assign V1 pattern
    for (int i = 0; i < cktin.size(); i++)
    {
        // for every input, set its value to current vector value
        cktin[i]->value = ctoi(v[i]);
    }
    for (int i = 0; i < sort_wlist.size(); i++)
    {
        if (i < cktin.size())
        {
            // in input wire list
            sort_wlist[i]->set_changed();
        }
        else
        {
            // remaining set unknown
            sort_wlist[i]->value = U;
        }
    }
    // do the simulation
    sim();
    // check activated
    for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos)
    {
        f = *pos;
        if (f->fault_type == sort_wlist[f->to_swlist]->value)
        {
            f->activate = TRUE;
        }
        else
            f->activate = FALSE;
    }
    // obtain V2
    tdf_sim_vector_two(v, num_of_current_detect);
}
// refer to faultsim.cpp
void ATPG::tdf_sim_vector_two(const string &v, int &num_of_current_detect)
{
    wptr w, faulty_wire;
    fptr simulated_fault_list[num_of_pattern];
    fptr f;
    int fault_type;
    int i, start_wire_index, nckt;
    int cnt_fault;
    // counts the number of faults
    cnt_fault = 0;

    // num_of_current_detect track undetected faults detected by vector
    num_of_current_detect = 0;

    // track minimum wire index of 16 faults
    start_wire_index = 10240;

    // for every input, set its value to current vector value
    for (i = 0; i < cktin.size(); i++)
    {
        if (i == 0)
            cktin[i]->value = ctoi(v[cktin.size()]);
        else
            cktin[i]->value = ctoi(v[i - 1]);
    }
    // assign V2 pattern
    nckt = sort_wlist.size();
    for (i = 0; i < nckt; i++)
    {
        if (i < cktin.size())
        {
            // in input wire list
            sort_wlist[i]->set_changed();
        }
        else
        {
            // remaining set unknown
            sort_wlist[i]->value = U;
        }
    }
    // do the simulation
    sim();
    // store fault-free 0,1,2 value in
    // wire_value1 (good value) and wire_value2 (faulty value)
    for (i = 0; i < nckt; i++)
    {
        switch (sort_wlist[i]->value)
        {
        case 1: // 1
            sort_wlist[i]->wire_value1 = ALL_ONE;
            sort_wlist[i]->wire_value2 = ALL_ONE;
            break;
        case 2: // unknown
            sort_wlist[i]->wire_value1 = 0x55555555;
            sort_wlist[i]->wire_value2 = 0x55555555;
            break;
        case 0: // 0
            sort_wlist[i]->wire_value1 = ALL_ZERO;
            sort_wlist[i]->wire_value2 = ALL_ZERO;
            break;
        }
    }

    // check undetected fault
    for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos)
    {
        int fault_detected[num_of_pattern] = {0};
        f = *pos;
        if (f->detect == REDUNDANT)
        {
            // ignore redundant
            continue;
        }
        if (f->activate == FALSE)
        {
            if ((next(pos, 1) == flist_undetect.cend()) && cnt_fault > 0)
            {
                // do fault simulation
                goto do_fsim;
            }
            else
            {
                continue;
            }
        }
        // consider excited fault
        // (sa1 with 0 or sa0 with 1)
        if (f->fault_type != sort_wlist[f->to_swlist]->value)
        {
            // if f is a PO or is
            // directly connected to an primary output
            // then detected
            if ((f->node->type == OUTPUT) ||
                (f->io == GO && sort_wlist[f->to_swlist]->is_output()))
            {
                f->detect = TRUE;
            }
            else
            {
                // if f is a GO
                if (f->io == GO)
                {
                    // if not faulty, mark as faulty
                    // then insert the corresponding wire to the list of faulty wires
                    if (!(sort_wlist[f->to_swlist]->is_faulty()))
                    {
                        sort_wlist[f->to_swlist]->set_faulty();
                        wlist_faulty.push_front(sort_wlist[f->to_swlist]);
                    }
                    // add to simulated fault list and inject
                    simulated_fault_list[cnt_fault] = f;
                    inject_fault_value(sort_wlist[f->to_swlist], cnt_fault, f->fault_type);
                    // mark it injected and schedule the outputs
                    sort_wlist[f->to_swlist]->set_fault_injected();
                    for (auto pos_n = sort_wlist[f->to_swlist]->onode.cbegin(), end_n = sort_wlist[f->to_swlist]->onode.cend(); pos_n != end_n; ++pos_n)
                    {
                        (*pos_n)->owire.front()->set_scheduled();
                    }
                    // count simulated faults
                    cnt_fault++;
                    // start_wire_indextrack smallest level of fault
                    start_wire_index = min(start_wire_index, f->to_swlist);
                }
                // if f is a GI
                else
                {
                    // if can propagate
                    // set faulty_wire equal to the faulty wire
                    faulty_wire = get_faulty_wire(f, fault_type);
                    if (faulty_wire != nullptr)
                    {
                        // if faulty_wire is PO, then detected
                        if (faulty_wire->is_output())
                        {
                            f->detect = TRUE;
                        }
                        else
                        {
                            // if not faulty, mark as faulty
                            // then insert the corresponding wire to the list of faulty wires
                            if (!(faulty_wire->is_faulty()))
                            {
                                faulty_wire->set_faulty();
                                wlist_faulty.push_front(faulty_wire);
                            }
                            // add to simulated fault list and inject
                            simulated_fault_list[cnt_fault] = f;
                            inject_fault_value(faulty_wire, cnt_fault, fault_type);
                            // mark it injected and schedule the outputs
                            faulty_wire->set_fault_injected();
                            for (auto pos_n = faulty_wire->onode.cbegin(), end_n = faulty_wire->onode.cend();
                                 pos_n != end_n; ++pos_n)
                            {
                                (*pos_n)->owire.front()->set_scheduled();
                            }
                            // count simulated faults
                            cnt_fault++;
                            // start_wire_indextrack smallest level of fault
                            start_wire_index = min(start_wire_index, f->to_swlist);
                        }
                    }
                }
            }
        }
        // do fault simulation
        if ((cnt_fault == num_of_pattern) || (next(pos, 1) == flist_undetect.cend()))
        {
        do_fsim:
            // start with start_wire_index, evaulate scheduled wires
            for (i = start_wire_index; i < nckt; i++)
            {
                if (sort_wlist[i]->is_scheduled())
                {
                    sort_wlist[i]->remove_scheduled();
                    fault_sim_evaluate(sort_wlist[i]);
                }
            }

            // pop out faulty wires from the wlist_faulty
            // if PO's value is different from good PO's, and it is not unknown, then detected
            while (!wlist_faulty.empty())
            {
                w = wlist_faulty.front();
                wlist_faulty.pop_front();
                w->remove_faulty();
                w->remove_fault_injected();
                w->set_fault_free();
                if (w->is_output())
                { // if primary output
                    for (i = 0; i < cnt_fault; i++)
                    { // check every undetected fault
                        if (!(simulated_fault_list[i]->detect))
                        {
                            if ((w->wire_value2 & Mask[i]) ^ // if value1 != value2
                                (w->wire_value1 & Mask[i]))
                            {
                                if (((w->wire_value2 & Mask[i]) ^ Unknown[i]) && // and not unknowns
                                    ((w->wire_value1 & Mask[i]) ^ Unknown[i]))
                                {
                                    fault_detected[i] = 1; // then the fault is detected
                                }
                            }
                        }
                    }
                }
                w->wire_value2 = w->wire_value1; // reset to fault-free values
            }                                    // pop out all faulty wires
            for (i = 0; i < cnt_fault; i++)
            {
                if (fault_detected[i] == 1)
                {
                    simulated_fault_list[i]->detect = TRUE;
                }
            }
            cnt_fault = 0;            // reset the counter of faults
            start_wire_index = 10240; // reset to a large value again
        }
    }

    /* fault dropping  */
    flist_undetect.remove_if(
        [&](const fptr fptr_ele)
        {
            if (fptr_ele->detect == TRUE)
            {
                num_of_current_detect += fptr_ele->eqv_fault_num;
                return true;
            }
            else
            {
                return false;
            }
        });
}
// refer to init_flist.cpp
void ATPG::generate_tdfault_list()
{
    int fault_num = 0;
    num_of_tdf_fault = 0;
    wptr w;
    nptr n;
    fptr_s f;
    /* walk through every wire in the circuit*/
    for (auto pos = sort_wlist.crbegin(); pos != sort_wlist.crend(); ++pos)
    {
        w = *pos;
        n = w->inode.front();
        /* for each gate, create a gate output stuck-at zero (SA0) fault */
        f = move(fptr_s(new (nothrow) FAULT));
        if (f == nullptr)
            error("No more room!");
        f->node = n;
        f->io = GO;
        f->fault_type = STR;
        f->to_swlist = w->wlist_index;
        f->detected_time = 0;
        /* for AND NOR NOT BUF, their GI fault is equivalent to GO SA0 fault */
        switch (n->type)
        {
        case NOT:
        case BUF:
            f->eqv_fault_num = 1;
            for (wptr wptr_ele : w->inode.front()->iwire)
            {
                if (wptr_ele->onode.size() > 1)
                    f->eqv_fault_num++;
            }
            break;
        case AND:
        case NOR:
        case INPUT:
        case OR:
        case NAND:
        case EQV:
        case XOR:
            f->eqv_fault_num = 1;
            break;
        }
        num_of_gate_fault += f->eqv_fault_num; // accumulate total fault count
        flist_undetect.push_front(f.get());    // initial undetected fault list contains all faults
        flist.push_front(move(f));             // push into the fault list
        /* for each gate, create a gate output stuck-at one (SA1) fault */
        f = move(fptr_s(new (nothrow) FAULT));
        if (f == nullptr)
            error("No more room!");
        f->node = n;
        f->io = GO;
        f->fault_type = STF;
        f->to_swlist = w->wlist_index;
        f->detected_time = 0;
        /* for OR NAND NOT BUF, their GI fault is equivalent to GO SA1 fault */
        switch (n->type)
        {
        case NOT:
        case BUF:
            f->eqv_fault_num = 1;
            for (wptr wptr_ele : w->inode.front()->iwire)
            {
                if (wptr_ele->onode.size() > 1)
                    f->eqv_fault_num++;
            }
            break;
        case OR:
        case NAND:
        case INPUT:
        case AND:
        case NOR:
        case EQV:
        case XOR:
            f->eqv_fault_num = 1;
            break;
        }
        num_of_gate_fault += f->eqv_fault_num;
        flist_undetect.push_front(f.get());
        flist.push_front(move(f));
        /*if w has multiple fanout branches */
        if (w->onode.size() > 1)
        {
            for (nptr nptr_ele : w->onode)
            {
                /* create SA0 for OR NOR EQV XOR gate inputs  */
                switch (nptr_ele->type)
                {
                case OUTPUT:
                case OR:
                case NOR:
                case AND:
                case NAND:
                case EQV:
                case XOR:
                    f = move(fptr_s(new (nothrow) FAULT));
                    if (f == nullptr)
                        error("No more room!");
                    f->node = nptr_ele;
                    f->io = GI;
                    f->fault_type = STR;
                    f->to_swlist = w->wlist_index;
                    f->eqv_fault_num = 1;
                    f->detected_time = 0;
                    /* f->index is the index number of gate input,
                       which GI fault is associated with*/
                    for (int k = 0; k < nptr_ele->iwire.size(); k++)
                    {
                        if (nptr_ele->iwire[k] == w)
                            f->index = k;
                    }
                    num_of_gate_fault++;
                    flist_undetect.push_front(f.get());
                    flist.push_front(move(f));
                    break;
                }
                /* create SA1 for AND NAND EQV XOR gate inputs  */
                switch (nptr_ele->type)
                {
                case OUTPUT:
                case OR:
                case NOR:
                case AND:
                case NAND:
                case EQV:
                case XOR:
                    f = move(fptr_s(new (nothrow) FAULT));
                    if (f == nullptr)
                        error("No more room!");
                    f->node = nptr_ele;
                    f->io = GI;
                    f->fault_type = STF;
                    f->to_swlist = w->wlist_index;
                    f->eqv_fault_num = 1;
                    f->detected_time = 0;
                    for (int k = 0; k < nptr_ele->iwire.size(); k++)
                    {
                        if (nptr_ele->iwire[k] == w)
                            f->index = k;
                    }
                    num_of_gate_fault++;
                    flist_undetect.push_front(f.get());
                    flist.push_front(move(f));
                    break;
                }
            }
        }
    }
    /*walk through all fautls, assign fault_no one by one  */
    for (fptr f : flist_undetect)
    {
        f->fault_no = fault_num;
        fault_num++;
        num_of_tdf_fault += f->eqv_fault_num;
    }
}