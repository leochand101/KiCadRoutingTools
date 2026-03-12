# Route ground plane
python3 route_planes.py ../Inflex_sense.kicad_pcb Inflex_sense.planes.kicad_pcb --nets 'GND' --plane-layers B.Cu


# Route SPI signals only 
#python3 route.py ../Inflex_sense.kicad_pcb Inflex_sense.routed.kicad_pcb --nets "/FSPICLK_SCLK" "/FSPID_MOSI" "/FSPIQ_MISO" "/FPSPICS2_CS" --layers F.Cu In1.Cu In2.Cu B.Cu

# Route all signal nets
#python3 route.py Inflex_sense.planes.kicad_pcb Inflex_sense.routed.kicad_pcb --nets "*SPI*" "*GPIO*" "*ADC*" "/Aintout" "/REGout" --layers F.Cu In1.Cu In2.Cu B.Cu 

# Route all nets
python3 route.py Inflex_sense.planes.kicad_pcb Inflex_sense.routed.kicad_pcb --nets "*SPI*" "*GPIO*" "*ADC*" "/Aintout" "/REGout" "*5V*" "*3p3*" --layers F.Cu In1.Cu In2.Cu B.Cu 

# Check connectivity
python3 check_connected.py Inflex_sense.routed.kicad_pcb

# Check for DRC violations (default clearance: 0.2mm)
python3 check_drc.py Inflex_sense.routed.kicad_pcb 
