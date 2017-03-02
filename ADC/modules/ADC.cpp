For reading 

  high = (inp(motorx->LM629PortLocation + LM629DataRegOffset) << 8);

  low  = (inp(motorx->LM629PortLocation + LM629DataRegOffset)) & 0x00ff;

  return (high | low);



For Writing 

  outp((motorx->LM629PortLocation + LM629DataRegOffset), (data >> 8));

  outp((motorx->LM629PortLocation + LM629DataRegOffset), data);

}
