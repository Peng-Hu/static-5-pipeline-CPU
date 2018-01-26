# static-5-pipeline-CPU

Thanks to my teammate, Linfeng Dong

There two version of my CPU, and I list their modules respectly as follows:
  1.  Static-5-pipline, intruction prefetch, interrupt handler, data forward unit, mutiplying unit, divider.
  2.  Static-5-pipline, axi unit(transaction between ram, board and cpu), interrupt handler, data forward unit, mutiplying unit, divider.
Beisides, in order to accelerate fetching data & instructions, I integrate data & instruction cache in axi unit.

The first version can run under 120Mhz. 
The second version can run >= 50Mhz
