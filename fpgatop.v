module top(phi, dac_vid, hsync, vsync);
  input phi;
  output [11:0] dac_vid;
  output hsync, vsync;
  
  wire valid;
  wire [6:0] charsel;
  
  harmonica   h(phi, charsel, valid);
  chdl_design v(phi, charsel, valid, dac_vid, hsync, vsync);
endmodule
