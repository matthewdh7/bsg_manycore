/**
 *    bsg_mesh_router_decoder_dor.v
 *
 *    Dimension ordered routing decoder
 *    
 *    depopulated ruche router.
 */

`include "bsg_defines.v"
`define DECODER_X_MAX 15
`define DECODER_X_MIN 8
`define X_HALF ((`DECODER_X_MAX + 1) - (`DECODER_X_MIN)) / 2

module bsg_mesh_router_decoder_dor
  import bsg_noc_pkg::*;
  import bsg_mesh_router_pkg::*;
  #(parameter `BSG_INV_PARAM(x_cord_width_p )
    , parameter `BSG_INV_PARAM(y_cord_width_p )
    , parameter dims_p = 2
    , parameter dirs_lp = (2*dims_p)+1
    , parameter ruche_factor_X_p=0
    , parameter ruche_factor_Y_p=0
    // XY_order_p = 1 :  X then Y
    // XY_order_p = 0 :  Y then X
    , parameter XY_order_p = 1
    , parameter depopulated_p = 1
    , parameter from_p = {dirs_lp{1'b0}}  // one-hot, indicates which direction is the input coming from.

    , parameter debug_p = 1
  )
  (
    input clk_i         // debug only
    , input reset_i     // debug only

    //, input v_i

    , input [x_cord_width_p-1:0] x_dirs_i
    , input [y_cord_width_p-1:0] y_dirs_i

    , input [x_cord_width_p-1:0] my_x_i
    , input [y_cord_width_p-1:0] my_y_i

    , output [dirs_lp-1:0] req_o
  );


  // check parameters

  // synopsys translate_off
  initial begin
    if (ruche_factor_X_p > 0) begin
      assert(dims_p > 2) else $fatal(1, "ruche in X direction requires dims_p greater than 2.");
    end
    
    if (ruche_factor_Y_p > 0) begin
      assert(dims_p > 3) else $fatal(1, "ruche in Y direction requires dims_p greater than 3.");
    end

    assert($countones(from_p) == 1) else $fatal(1, "Must define from_p as one-hot value.");

    assert(ruche_factor_X_p < (1<<x_cord_width_p)) else $fatal(1, "ruche factor in X direction is too large");
    assert(ruche_factor_Y_p < (1<<y_cord_width_p)) else $fatal(1, "ruche factor in Y direction is too large");
  end
  // synopsys translate_on




  // compare coordinates
  wire x_eq = (x_dirs_i == my_x_i);
  wire y_eq = (y_dirs_i == my_y_i);
  wire x_gt = x_dirs_i > my_x_i;
  wire y_gt = y_dirs_i > my_y_i;
  wire x_lt = ~x_gt & ~x_eq;
  wire y_lt = ~y_gt & ~y_eq;

  // valid signal
  logic [dirs_lp-1:0] req;
  assign req_o = req;


  // P-port
  assign req[P] = x_eq & y_eq;

  // definitions
  wire [x_cord_width_p-1:0] x_cord_max = (x_cord_width_p)'(`DECODER_X_MAX);
  wire [x_cord_width_p-1:0] x_cord_min = (x_cord_width_p)'(`DECODER_X_MIN);

  wire [x_cord_width_p-1:0] mesh_dist = x_gt ? (x_dirs_i - my_x_i) : (my_x_i - x_dirs_i);
  wire [x_cord_width_p-1:0] torus_dist = (x_cord_max - x_cord_min) + 1'b1 - mesh_dist;

  // distances in hops
  wire [x_cord_width_p-1:0] mesh_hops = ((mesh_dist - 1'b1) / 'd3) + ((mesh_dist - 1) % 'd3) + 1'b1;

  // dist from my_x_i to R/L edge, +1 to get around torus link
  wire [x_cord_width_p-1:0] src_over_edge = x_gt ? (((my_x_i - x_cord_min) / 'd3) + ((my_x_i - x_cord_min) % 3) + 1'b1) :
                                                   (((x_cord_max - my_x_i) / 'd3) + ((x_cord_max - my_x_i) % 3) + 1'b1);
  // dist from closest edge to x_dirs_i
  wire [x_cord_width_p-1:0] edge_to_dest = (x_dirs_i == x_cord_min) ? '0 : 
                                           (x_dirs_i == x_cord_max) ? '0 :
                                            x_gt ? (((x_cord_max - x_dirs_i - 1'b1) / 'd3) + ((x_cord_max - x_dirs_i - 1'b1) % 3) + 1'b1) :
                                                   (((x_dirs_i - x_cord_min - 1'b1) / 'd3) + ((x_dirs_i - x_cord_min - 1'b1) % 3) + 1'b1);

  wire [x_cord_width_p-1:0] torus_hops = src_over_edge + edge_to_dest;

  // for rev network
  wire [x_cord_width_p-1:0] mesh_hops_rev = ((mesh_dist) / 'd3) + ((mesh_dist) % 'd3);
  wire [x_cord_width_p-1:0] edge_to_dest_rev = (x_dirs_i == x_cord_min) ? '0 : 
                                           (x_dirs_i == x_cord_max) ? '0 :
                                            x_gt ? (((x_cord_max - x_dirs_i) / 'd3) + ((x_cord_max - x_dirs_i) % 3)) :
                                                   (((x_dirs_i - x_cord_min) / 'd3) + ((x_dirs_i - x_cord_min) % 3));

  wire [x_cord_width_p-1:0] torus_hops_rev = src_over_edge + edge_to_dest_rev;
 

  wire take_torus = torus_hops < mesh_hops;
  wire take_torus_rev = torus_hops_rev < mesh_hops_rev;

  wire send_left = take_torus ? x_gt : x_lt; // OH: if take_torus: if x_gt, go left, if x_lt go right
  wire send_right = take_torus ? x_lt : x_gt;

  wire send_left_rev = take_torus_rev ? x_gt : x_lt;
  wire send_right_rev = take_torus_rev ? x_lt : x_gt;


  if (ruche_factor_X_p > 0) begin
    wire [x_cord_width_p-1:0] ruche_factor_X_lp = (x_cord_width_p)'(ruche_factor_X_p); //MH: added to abbreviate text

    if (XY_order_p) begin
      // make sure there is no under/overflow.
      wire [x_cord_width_p:0] re_cord_og = (x_cord_width_p+1)'(my_x_i + ruche_factor_X_p); //MH: how far right the ruche east link would go

      wire send_rw, send_re;
      wire send_rw_og, send_re_og;
      wire [x_cord_width_p-1:0] re_cord = (my_x_i > (x_cord_max - ruche_factor_X_lp)) ? ((ruche_factor_X_lp - (x_cord_max - my_x_i) - 1'b1) + x_cord_min) : my_x_i + ruche_factor_X_lp;
      wire [x_cord_width_p-1:0] rw_cord = (my_x_i < (ruche_factor_X_lp + x_cord_min)) ? (x_cord_max + 1'b1 - ((ruche_factor_X_lp + x_cord_min) - my_x_i)) : my_x_i - ruche_factor_X_lp; 

      if (depopulated_p) begin
        // assign send_rw_og = (my_x_i > (x_cord_width_p)'(ruche_factor_X_p)) && (x_dirs_i < (my_x_i - (x_cord_width_p)'(ruche_factor_X_p)));
        // assign send_re_og = !re_cord[x_cord_width_p] && (x_dirs_i > re_cord[0+:x_cord_width_p]);

        assign send_rw = send_left & (take_torus ? (torus_dist > 'd3) : (rw_cord > x_dirs_i)) & (my_x_i >= (x_cord_min + 'd3));
        assign send_re = send_right & (take_torus ? (torus_dist > 'd3) : (re_cord < x_dirs_i)) & (my_x_i <= (x_cord_max - 'd3));
      end
      else begin
        assign send_rw = (my_x_i >= (x_cord_width_p)'(ruche_factor_X_p)) && (x_dirs_i <= (my_x_i - (x_cord_width_p)'(ruche_factor_X_p)));
        assign send_re = !re_cord_og[x_cord_width_p] && (x_dirs_i >= re_cord_og[0+:x_cord_width_p]); // check no overflow
      end

      assign req[W]  = send_left & ~send_rw;
      assign req[RW] = send_rw;
      assign req[E]  = send_right & ~send_re;
      assign req[RE] = send_re;

      always_ff @(negedge clk_i) begin
          if (~reset_i) begin
              if (~(^my_x_i === 1'bX) & ~(^x_dirs_i === 1'bX)) begin
              assert(x_eq ^ (send_left | send_right))
                  else $fatal(1, "CANNOT HAVE X_EQ AND SENDING L/R");
              end
              if (~(send_rw === 1'bX) & ~(send_re === 1'bX) & ~(send_left === 1'bX) & ~(send_right === 1'bX)) begin
                  assert((send_rw & send_left) | (~send_rw))
                      else begin 
                          $display("torus: %d, my_x|x_dest: %d|%d, send_rw|re: %d|%d, send_left|right: %d|%d", take_torus, my_x_i, x_dirs_i, send_rw, send_re, send_left, send_right);
                          $fatal(1, "-----WARNING: sending on west ruche without send_left");
                      end
                  assert((send_re & send_right) | (~send_re))
                      else begin
                          $display("torus: %d, my_x|x_dest: %d|%d, send_rw|re: %d|%d, send_left|right: %d|%d", take_torus, my_x_i, x_dirs_i, send_rw, send_re, send_left, send_right);
                          $fatal(1, "-----WARNING: sending on east ruche without send_right");
                      end
              end
          end
      end

    end
    else begin //MH: rev network

      wire [x_cord_width_p-1:0] dxp = (x_cord_width_p)'((x_dirs_i - my_x_i) % ruche_factor_X_p);
      wire [x_cord_width_p-1:0] dxn = (x_cord_width_p)'((my_x_i - x_dirs_i) % ruche_factor_X_p);

      if (from_p[S] | from_p[N] | from_p[P]) begin //MH: coming from y, turning (can't turn right onto ruche)
        if (depopulated_p) begin
          // assign req[W]  = y_eq & x_lt;
          // assign req[RW] = 1'b0;
          // assign req[E]  = y_eq & x_gt;
          // assign req[RE] = 1'b0;
          assign req[W]   = y_eq & send_left_rev;
          assign req[RW]  = 1'b0;
          assign req[E]   = y_eq & send_right_rev;
          assign req[RE]  = 1'b0;
        end
        else begin //MH: unused
          assign req[W]  = y_eq & x_lt & (dxn != '0);
          assign req[RW] = y_eq & x_lt & (dxn == '0);
          assign req[E]  = y_eq & x_gt & (dxp != '0);
          assign req[RE] = y_eq & x_gt & (dxp == '0);
        end
      end
      else if(from_p[W]) begin
        // assign req[RE] = y_eq & x_gt & (dxp == '0);
        // assign req[E]  = y_eq & x_gt & (dxp != '0);
        // assign req[RW] = 1'b0;
        // assign req[W]  = 1'b0;
          assign req[W]   = 1'b0;
          assign req[RW]  = 1'b0;
          assign req[E]   = y_eq & (take_torus_rev ?  send_right :
                                                      x_gt & (dxp != '0));
          assign req[RE]  = y_eq & (take_torus_rev ?  (src_over_edge - 1'b1) % 3 == '0 :
                                                      x_gt & (dxp == '0));
      end
      else if (from_p[E]) begin
        // assign req[RE] = 1'b0;
        // assign req[E]  = 1'b0;
        // assign req[RW] = y_eq & x_lt & (dxn == '0);
        // assign req[W]  = y_eq & x_lt & (dxn != '0);
        assign req[W]   = y_eq & (take_torus_rev ?  send_left :
                                                    x_lt & (dxn != '0));
        assign req[RW]  = y_eq & (take_torus_rev ?  (src_over_edge - 1'b1) % 3 == '0 :
                                                    x_lt & (dxn == '0));
        assign req[E]   = 1'b0;
        assign req[RE]  = 1'b0;
      end
      else if (from_p[RW]) begin
        // assign req[RE] = y_eq & x_gt;
        // assign req[E]  = 1'b0;
        // assign req[RW] = 1'b0;
        // assign req[W]  = 1'b0;
        assign req[RE] = y_eq & (take_torus_rev ? my_x_i != x_cord_max :
                                                  1'b0);
        assign req[E]  = y_eq & (take_torus_rev ? my_x_i == x_cord_max :
                                                  1'b0);
        assign req[RW] = 1'b0;
        assign req[W]  = 1'b0;
      end
      else if (from_p[RE]) begin
        // assign req[RE] = 1'b0;
        // assign req[E]  = 1'b0;
        // assign req[RW] = y_eq & x_lt;
        // assign req[W]  = 1'b0;
        assign req[RE] = 1'b0;
        assign req[E]  = 1'b0;
        assign req[RW] = y_eq & (take_torus_rev ? my_x_i != x_cord_min :
                                                  1'b0);
        assign req[W]  = y_eq & (take_torus_rev ? my_x_i != x_cord_min :
                                                  1'b0);
      end
      else if (from_p[RN] | from_p[RS]) begin
        if (depopulated_p) begin
          // If depopulated, there wouldn't be these paths.
          assign req[RE] = 1'b0;
          assign req[E]  = 1'b0;
          assign req[RW] = 1'b0;
          assign req[W]  = 1'b0;
        end
        else begin //MH: unused
          assign req[W]  = y_eq & x_lt & (dxn != '0);
          assign req[RW] = y_eq & x_lt & (dxn == '0);
          assign req[E]  = y_eq & x_gt & (dxp != '0);
          assign req[RE] = y_eq & x_gt & (dxp == '0);
        end
      end
    end
  end
  else begin
    if (XY_order_p) begin
      assign req[W] = x_lt;
      assign req[E] = x_gt;
    end
    else begin
      assign req[W] = y_eq & x_lt;
      assign req[E] = y_eq & x_gt;
    end
  end


  
  if (ruche_factor_Y_p > 0) begin
    if (XY_order_p == 0) begin
      // make sure there is no under/overflow.
      wire [y_cord_width_p:0] rs_cord = (y_cord_width_p+1)'(my_y_i + ruche_factor_Y_p);
      wire send_rn, send_rs;

      if (depopulated_p) begin
        assign send_rn = (my_y_i > (y_cord_width_p)'(ruche_factor_Y_p)) && (y_dirs_i < (my_y_i - (y_cord_width_p)'(ruche_factor_Y_p)));
        assign send_rs = !rs_cord[y_cord_width_p] && (y_dirs_i > rs_cord[0+:y_cord_width_p]);
      end
      else begin
        assign send_rn = (my_y_i >= (y_cord_width_p)'(ruche_factor_Y_p)) && (y_dirs_i <= (my_y_i - (y_cord_width_p)'(ruche_factor_Y_p)));
        assign send_rs = !rs_cord[y_cord_width_p] && (y_dirs_i >= rs_cord[0+:y_cord_width_p]);
      end

      assign req[N]  = y_lt & ~send_rn;
      assign req[RN] = send_rn;
      assign req[S]  = y_gt & ~send_rs;
      assign req[RS] = send_rs;
    end
    else begin

      wire [y_cord_width_p-1:0] dyp = (y_cord_width_p)'((y_dirs_i - my_y_i) % ruche_factor_Y_p);
      wire [y_cord_width_p-1:0] dyn = (y_cord_width_p)'((my_y_i - y_dirs_i) % ruche_factor_Y_p);

      if (from_p[E] | from_p[W] | from_p[P]) begin
        if (depopulated_p) begin
          assign req[N]  = x_eq & y_lt;
          assign req[RN] = 1'b0;
          assign req[S]  = x_eq & y_gt;
          assign req[RS] = 1'b0;
        end
        else begin
          assign req[N]  = x_eq & y_lt & (dyn != '0);
          assign req[RN] = x_eq & y_lt & (dyn == '0);
          assign req[S]  = x_eq & y_gt & (dyp != '0);
          assign req[RS] = x_eq & y_gt & (dyp == '0);
        end
      end
      else if (from_p[N]) begin
        assign req[RS] = x_eq & y_gt & (dyp == '0);
        assign req[S]  = x_eq & y_gt & (dyp != '0);
        assign req[RN] = 1'b0;
        assign req[N]  = 1'b0;
      end
      else if (from_p[S]) begin
        assign req[RS] = 1'b0;
        assign req[S]  = 1'b0;
        assign req[RN] = x_eq & y_lt & (dyn == '0);
        assign req[N]  = x_eq & y_lt & (dyn != '0);
      end
      else if (from_p[RN]) begin
        assign req[RS] = x_eq & y_gt;
        assign req[S]  = 1'b0;
        assign req[RN] = 1'b0;
        assign req[N]  = 1'b0;
      end
      else if (from_p[RS]) begin
        assign req[RS] = 1'b0;
        assign req[S]  = 1'b0;
        assign req[RN] = x_eq & y_lt;
        assign req[N]  = 1'b0;
      end
      else if (from_p[RW] | from_p[RE]) begin
        if (depopulated_p) begin
          // If depopulated, there wouldn't be these paths.
          assign req[RS] = 1'b0;
          assign req[S]  = 1'b0;
          assign req[RN] = 1'b0;
          assign req[N]  = 1'b0;
        end
        else begin
          assign req[N]  = x_eq & y_lt & (dyn != '0);
          assign req[RN] = x_eq & y_lt & (dyn == '0);
          assign req[S]  = x_eq & y_gt & (dyp != '0);
          assign req[RS] = x_eq & y_gt & (dyp == '0);
        end
      end
    end

  end
  else begin
    if (XY_order_p == 0) begin
      assign req[N] = y_lt;
      assign req[S] = y_gt;
    end
    else begin
      assign req[N] = x_eq & y_lt;
      assign req[S] = x_eq & y_gt;
    end
  end


  // synopsys translate_off
  if (debug_p) begin
    always_ff @ (negedge clk_i) begin
      if (~reset_i) begin
        assert($countones(req_o) < 2)
          else $fatal(1, "multiple req_o detected. %b", req_o);
      end
    end
  end
  else begin
    wire unused0 = clk_i;
    wire unused1 = reset_i;
  end
  // synopsys translate_on




endmodule

`BSG_ABSTRACT_MODULE(bsg_mesh_router_decoder_dor)

