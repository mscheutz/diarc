object(self, actor).
%sees(object_1, banana).
%sees(object_2, cucumber).
%sees(object_3, juice).
%sees(object_4, bread).
%sees(object_5, orange).
%sees(object_6, milk).
%sees(object_7, apple).

free(self).
%athuman(lunchbox).
%atrobot(object_1).
%atrobot(object_2).
%atrobot(object_3).
%atrobot(object_4).
%atrobot(object_5).
%atrobot(object_6).
%atrobot(object_7).

%and(contains(box,object_1), contains(box,object_4), contains(box,object_7), athuman(box), free(self))
