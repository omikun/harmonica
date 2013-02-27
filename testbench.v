module top();
  reg phi, togclk;

  wire [6:0] charsel, text, wrcol;
  wire hsync, vsync, valid;
  wire [11:0] dac_vid;

  harmonica   h(phi, charsel, valid);
  chdl_design v(phi, charsel, valid, dac_vid, hsync, vsync);
   
  initial
    begin
       $dumpfile("dump.vcd");
       $dumpvars(1, top);
       phi = 0; togclk = 0;
       #9 togclk = 1;
       #99991 $finish;
    end

   always
     begin
       #5 phi = phi ^ togclk;
     end
endmodule // top
