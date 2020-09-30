<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.3">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="no" active="no"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="100" name="PaJa" color="12" fill="7" visible="yes" active="yes"/>
<layer number="101" name="Doplnky" color="5" fill="1" visible="yes" active="yes"/>
<layer number="102" name="Kola" color="11" fill="7" visible="yes" active="yes"/>
<layer number="103" name="Popisy" color="2" fill="8" visible="yes" active="yes"/>
<layer number="104" name="Zapojeni" color="6" fill="7" visible="yes" active="yes"/>
<layer number="151" name="HeatSink" color="7" fill="1" visible="no" active="no"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="no" active="no"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="no"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="OrgLBR" color="13" fill="1" visible="no" active="no"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="atmel">
<description>&lt;b&gt;AVR Devices&lt;/b&gt;&lt;p&gt;
Configurable logic, microcontrollers, nonvolatile memories&lt;p&gt;
Based on the following sources:&lt;p&gt;
&lt;ul&gt;
&lt;li&gt;www.atmel.com
&lt;li&gt;CD-ROM : Configurable Logic Microcontroller Nonvolatile Memory
&lt;li&gt;CadSoft download site, www.cadsoft.de or www.cadsoftusa.com , file at90smcu_v400.zip
&lt;li&gt;avr.lbr
&lt;/ul&gt;
&lt;author&gt;Revised by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="DIL28-3">
<description>&lt;B&gt;Dual In Line&lt;/B&gt; 0.3 inch</description>
<wire x1="-18.542" y1="-0.635" x2="-18.542" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="0.635" x2="-18.542" y2="-0.635" width="0.1524" layer="21" curve="-180"/>
<wire x1="-18.542" y1="-2.794" x2="18.542" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="2.794" x2="-18.542" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-18.542" y1="2.794" x2="18.542" y2="2.794" width="0.1524" layer="21"/>
<wire x1="18.542" y1="2.794" x2="18.542" y2="-2.794" width="0.1524" layer="21"/>
<pad name="1" x="-16.51" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-13.97" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-11.43" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="-8.89" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="-6.35" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="-3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="-1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="9" x="3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="10" x="6.35" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="11" x="8.89" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="12" x="11.43" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="13" x="13.97" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="14" x="16.51" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="15" x="16.51" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="16" x="13.97" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="17" x="11.43" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="18" x="8.89" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="19" x="6.35" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="20" x="3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="21" x="1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="22" x="-1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="23" x="-3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="24" x="-6.35" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="25" x="-8.89" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="26" x="-11.43" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="27" x="-13.97" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="28" x="-16.51" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-19.2024" y="-2.54" size="1.778" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-15.875" y="-0.635" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="23-I/O-2">
<wire x1="-17.78" y1="30.48" x2="20.32" y2="30.48" width="0.254" layer="94"/>
<wire x1="20.32" y1="30.48" x2="20.32" y2="-33.02" width="0.254" layer="94"/>
<wire x1="20.32" y1="-33.02" x2="-17.78" y2="-33.02" width="0.254" layer="94"/>
<wire x1="-17.78" y1="-33.02" x2="-17.78" y2="30.48" width="0.254" layer="94"/>
<text x="-17.78" y="-35.56" size="1.778" layer="95">&gt;NAME</text>
<text x="-17.78" y="31.75" size="1.778" layer="96">&gt;VALUE</text>
<pin name="PB5(SCK)" x="25.4" y="-30.48" length="middle" rot="R180"/>
<pin name="PB7(XTAL2/TOSC2)" x="-22.86" y="5.08" length="middle"/>
<pin name="PB6(XTAL1/TOSC1)" x="-22.86" y="10.16" length="middle"/>
<pin name="GND@1" x="-22.86" y="-2.54" length="middle" direction="pwr"/>
<pin name="VCC@1" x="-22.86" y="-7.62" length="middle" direction="pwr"/>
<pin name="GND" x="-22.86" y="22.86" length="middle" direction="pwr"/>
<pin name="AREF" x="-22.86" y="20.32" length="middle" direction="pas"/>
<pin name="AVCC" x="-22.86" y="17.78" length="middle" direction="pwr"/>
<pin name="PB4(MISO)" x="25.4" y="-27.94" length="middle" rot="R180"/>
<pin name="PB3(MOSI/OC2)" x="25.4" y="-25.4" length="middle" rot="R180"/>
<pin name="PB2(SS/OC1B)" x="25.4" y="-22.86" length="middle" rot="R180"/>
<pin name="PB1(OC1A)" x="25.4" y="-20.32" length="middle" rot="R180"/>
<pin name="PB0(ICP)" x="25.4" y="-17.78" length="middle" rot="R180"/>
<pin name="PD7(AIN1)" x="25.4" y="-12.7" length="middle" rot="R180"/>
<pin name="PD6(AIN0)" x="25.4" y="-10.16" length="middle" rot="R180"/>
<pin name="PD5(T1)" x="25.4" y="-7.62" length="middle" rot="R180"/>
<pin name="PD4(XCK/T0)" x="25.4" y="-5.08" length="middle" rot="R180"/>
<pin name="PD3(INT1)" x="25.4" y="-2.54" length="middle" rot="R180"/>
<pin name="PD2(INT0)" x="25.4" y="0" length="middle" rot="R180"/>
<pin name="PD1(TXD)" x="25.4" y="2.54" length="middle" rot="R180"/>
<pin name="PD0(RXD)" x="25.4" y="5.08" length="middle" rot="R180"/>
<pin name="PC5(ADC5/SCL)" x="25.4" y="15.24" length="middle" rot="R180"/>
<pin name="PC4(ADC4/SDA)" x="25.4" y="17.78" length="middle" rot="R180"/>
<pin name="PC3(ADC3)" x="25.4" y="20.32" length="middle" rot="R180"/>
<pin name="PC2(ADC2)" x="25.4" y="22.86" length="middle" rot="R180"/>
<pin name="PC1(ADC1)" x="25.4" y="25.4" length="middle" rot="R180"/>
<pin name="PC0(ADC0)" x="25.4" y="27.94" length="middle" rot="R180"/>
<pin name="PC6(/RESET)" x="-22.86" y="27.94" length="middle" function="dot"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MEGA8-P" prefix="IC">
<description>&lt;b&gt;MICROCONTROLLER&lt;/b&gt;&lt;p&gt;
8 Kbytes FLASH, 1 kbytes SRAM, 512 bytes EEPROM, USART, 4-channel 10 bit ADC, 2-channel 8 bit ADC&lt;br&gt;
Pin compatible with Atmega48, ATMega88, ATMega168&lt;br&gt;
Source: avr.lbr</description>
<gates>
<gate name="G$1" symbol="23-I/O-2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DIL28-3">
<connects>
<connect gate="G$1" pin="AREF" pad="21"/>
<connect gate="G$1" pin="AVCC" pad="20"/>
<connect gate="G$1" pin="GND" pad="22"/>
<connect gate="G$1" pin="GND@1" pad="8"/>
<connect gate="G$1" pin="PB0(ICP)" pad="14"/>
<connect gate="G$1" pin="PB1(OC1A)" pad="15"/>
<connect gate="G$1" pin="PB2(SS/OC1B)" pad="16"/>
<connect gate="G$1" pin="PB3(MOSI/OC2)" pad="17"/>
<connect gate="G$1" pin="PB4(MISO)" pad="18"/>
<connect gate="G$1" pin="PB5(SCK)" pad="19"/>
<connect gate="G$1" pin="PB6(XTAL1/TOSC1)" pad="9"/>
<connect gate="G$1" pin="PB7(XTAL2/TOSC2)" pad="10"/>
<connect gate="G$1" pin="PC0(ADC0)" pad="23"/>
<connect gate="G$1" pin="PC1(ADC1)" pad="24"/>
<connect gate="G$1" pin="PC2(ADC2)" pad="25"/>
<connect gate="G$1" pin="PC3(ADC3)" pad="26"/>
<connect gate="G$1" pin="PC4(ADC4/SDA)" pad="27"/>
<connect gate="G$1" pin="PC5(ADC5/SCL)" pad="28"/>
<connect gate="G$1" pin="PC6(/RESET)" pad="1"/>
<connect gate="G$1" pin="PD0(RXD)" pad="2"/>
<connect gate="G$1" pin="PD1(TXD)" pad="3"/>
<connect gate="G$1" pin="PD2(INT0)" pad="4"/>
<connect gate="G$1" pin="PD3(INT1)" pad="5"/>
<connect gate="G$1" pin="PD4(XCK/T0)" pad="6"/>
<connect gate="G$1" pin="PD5(T1)" pad="11"/>
<connect gate="G$1" pin="PD6(AIN0)" pad="12"/>
<connect gate="G$1" pin="PD7(AIN1)" pad="13"/>
<connect gate="G$1" pin="VCC@1" pad="7"/>
</connects>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="+5V">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+5V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="+5V" prefix="P+">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="+5V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="GND" prefix="GND">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="#PaJa_22">
<description>&lt;B&gt;PaJa 22&lt;/B&gt; - knihovna   &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; 
&lt;I&gt;(vytvoreno 1.5.2010)&lt;/I&gt;&lt;BR&gt;
Univerzální knihovna soucastek do Eagle &lt;I&gt;(od verze 4.13)&lt;/I&gt;&lt;BR&gt;
&lt;BR&gt;
Knihovna obsahuje: 300 soucastek na DPS, 400 do SCHematu&lt;BR&gt;
&lt;BR&gt;
&lt;Author&gt;Copyright (C) PaJa 2001-2010&lt;BR&gt;
http://www.paja-trb.unas.cz&lt;BR&gt;
paja-trb@seznam.cz
&lt;/author&gt;</description>
<packages>
<package name="C-EL_2">
<wire x1="-0.1378" y1="-1.02" x2="-0.1378" y2="-1.782" width="0.127" layer="21"/>
<wire x1="-0.5188" y1="-1.401" x2="0.2432" y2="-1.401" width="0.127" layer="21"/>
<wire x1="0.4762" y1="-1.524" x2="0.9842" y2="-1.524" width="0.127" layer="21"/>
<wire x1="0.9842" y1="-1.524" x2="0.9842" y2="1.524" width="0.127" layer="21"/>
<wire x1="0.9842" y1="1.524" x2="0.4762" y2="1.524" width="0.127" layer="21"/>
<wire x1="0.4762" y1="1.524" x2="0.4762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="1.4112" y1="1.5" x2="1.7862" y2="1.5" width="0.127" layer="21"/>
<wire x1="1.7862" y1="1.5" x2="1.7862" y2="-1.5" width="0.127" layer="21"/>
<wire x1="1.7862" y1="-1.5" x2="1.4112" y2="-1.5" width="0.127" layer="21"/>
<wire x1="1.4112" y1="-1.5" x2="1.4112" y2="1.5" width="0.127" layer="21"/>
<wire x1="1.5762" y1="0" x2="2.1962" y2="0" width="0.127" layer="21"/>
<wire x1="0.0262" y1="0" x2="0.2712" y2="0" width="0.127" layer="21"/>
<wire x1="0.2712" y1="0" x2="0.3362" y2="0" width="0.127" layer="21"/>
<wire x1="0.2712" y1="0" x2="0.439" y2="0" width="0.127" layer="21"/>
<circle x="1.1112" y="0" radius="2.4848" width="0.127" layer="21"/>
<circle x="-0.0018" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="2.2242" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="C+" x="0" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<pad name="C-" x="2.2225" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<text x="-1.2738" y="2.544" size="1.27" layer="25">&gt;Name</text>
<text x="-1.5918" y="-3.816" size="1.27" layer="27">&gt;Value</text>
<text x="0.6342" y="1.908" size="0.254" layer="100">PaJa</text>
<rectangle x1="1.4112" y1="-1.5" x2="1.7862" y2="1.425" layer="21"/>
</package>
<package name="C-EL_2,5">
<wire x1="-0.635" y1="-1.524" x2="-0.127" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.127" y1="-1.524" x2="-0.127" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.127" y1="1.524" x2="-0.635" y2="1.524" width="0.127" layer="21"/>
<wire x1="-1.868" y1="-0.861" x2="-1.868" y2="-1.623" width="0.127" layer="21"/>
<wire x1="-2.249" y1="-1.242" x2="-1.487" y2="-1.242" width="0.127" layer="21"/>
<wire x1="-0.635" y1="1.524" x2="-0.635" y2="-1.524" width="0.127" layer="21"/>
<wire x1="0.3" y1="1.5" x2="0.675" y2="1.5" width="0.127" layer="21"/>
<wire x1="0.675" y1="1.5" x2="0.675" y2="-1.5" width="0.127" layer="21"/>
<wire x1="0.675" y1="-1.5" x2="0.3" y2="-1.5" width="0.127" layer="21"/>
<wire x1="0.3" y1="-1.5" x2="0.3" y2="1.5" width="0.127" layer="21"/>
<wire x1="0.62" y1="0" x2="1.24" y2="0" width="0.127" layer="21"/>
<wire x1="-1.24" y1="0" x2="-0.8146" y2="0" width="0.127" layer="21"/>
<wire x1="-0.8146" y1="0" x2="-0.775" y2="0" width="0.127" layer="21"/>
<wire x1="-0.8146" y1="0" x2="-0.6758" y2="0" width="0.127" layer="21"/>
<circle x="0" y="0" radius="3.255" width="0.127" layer="21"/>
<circle x="-1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="C+" x="-1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<pad name="C-" x="1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<text x="-1.431" y="1.59" size="1.27" layer="25">&gt;Name</text>
<text x="-1.272" y="-2.862" size="1.27" layer="27">&gt;Value</text>
<text x="-0.2447" y="-0.4478" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="0.3" y1="-1.5" x2="0.675" y2="1.425" layer="21"/>
</package>
<package name="C-EL_3,5">
<wire x1="1.1113" y1="-1.524" x2="1.6193" y2="-1.524" width="0.127" layer="21"/>
<wire x1="1.6193" y1="-1.524" x2="1.6193" y2="1.524" width="0.127" layer="21"/>
<wire x1="1.6193" y1="1.524" x2="1.1113" y2="1.524" width="0.127" layer="21"/>
<wire x1="0.5013" y1="-0.853" x2="0.5013" y2="-1.615" width="0.127" layer="21"/>
<wire x1="0.1203" y1="-1.234" x2="0.8823" y2="-1.234" width="0.127" layer="21"/>
<wire x1="1.1113" y1="1.524" x2="1.1113" y2="-1.524" width="0.127" layer="21"/>
<wire x1="2.0463" y1="1.5" x2="2.4213" y2="1.5" width="0.127" layer="21"/>
<wire x1="2.4213" y1="1.5" x2="2.4213" y2="-1.5" width="0.127" layer="21"/>
<wire x1="2.4213" y1="-1.5" x2="2.0463" y2="-1.5" width="0.127" layer="21"/>
<wire x1="2.0463" y1="-1.5" x2="2.0463" y2="1.5" width="0.127" layer="21"/>
<wire x1="2.3623" y1="0" x2="2.9783" y2="0" width="0.127" layer="21"/>
<wire x1="0.4747" y1="0" x2="0.5063" y2="0" width="0.127" layer="21"/>
<wire x1="0.5063" y1="0" x2="0.9713" y2="0" width="0.127" layer="21"/>
<wire x1="0.5063" y1="0" x2="1.0705" y2="0" width="0.127" layer="21"/>
<circle x="1.7463" y="0.155" radius="4.03" width="0.127" layer="21"/>
<circle x="-0.0027" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="3.4953" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="C+" x="0" y="0" drill="0.8128" diameter="1.9304"/>
<pad name="C-" x="3.4925" y="0" drill="0.8128" diameter="1.9304" shape="square"/>
<text x="-0.6476" y="-2.9909" size="1.27" layer="27">&gt;VALUE</text>
<text x="4.7965" y="3.0261" size="1.27" layer="25" rot="R180">&gt;NAME</text>
<text x="2.7003" y="1.113" size="0.254" layer="100">PaJa</text>
<rectangle x1="2.0463" y1="-1.5" x2="2.4213" y2="1.425" layer="21"/>
</package>
<package name="C-EL_5">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="0" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="1.524" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-1.236" y2="0" width="0.127" layer="21"/>
<wire x1="-1.236" y1="0" x2="-1.524" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-2.003" y1="0" x2="-1.236" y2="0" width="0.127" layer="21"/>
<wire x1="1.55" y1="0" x2="2.003" y2="0" width="0.127" layer="21"/>
<wire x1="-1.905" y1="-0.9525" x2="-1.27" y2="-0.9525" width="0.127" layer="21"/>
<wire x1="-1.5875" y1="-0.635" x2="-1.5875" y2="-1.27" width="0.127" layer="21"/>
<circle x="0" y="0" radius="5.1308" width="0.127" layer="21"/>
<circle x="-2.54" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="C-" x="2.54" y="0" drill="0.8128" diameter="2.1844" shape="square"/>
<pad name="C+" x="-2.54" y="0" drill="0.8128" diameter="2.1844"/>
<text x="1.272" y="0.318" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="-3.017" y="1.758" size="1.4224" layer="25">&gt;Name</text>
<text x="-3.663" y="-3.186" size="1.4224" layer="27">&gt;Value</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL_5+">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="0" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="1.524" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-1.395" y2="0" width="0.127" layer="21"/>
<wire x1="-1.395" y1="0" x2="-1.524" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-2.003" y1="0" x2="-1.395" y2="0" width="0.127" layer="21"/>
<wire x1="1.55" y1="0" x2="2.003" y2="0" width="0.127" layer="21"/>
<wire x1="-2.019" y1="-1.248" x2="-1.081" y2="-1.248" width="0.127" layer="21"/>
<wire x1="-1.55" y1="-0.779" x2="-1.55" y2="-1.717" width="0.127" layer="21"/>
<circle x="0" y="0" radius="6.519" width="0.127" layer="21"/>
<circle x="-2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="C+" x="-2.54" y="0" drill="0.8128" diameter="2.54"/>
<pad name="C-" x="2.54" y="0" drill="0.8128" diameter="2.54" shape="square"/>
<text x="-4.299" y="-3.663" size="1.6764" layer="27">&gt;VALUE</text>
<text x="-3.653" y="1.758" size="1.6764" layer="25">&gt;NAME</text>
<text x="1.113" y="0.318" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL_7,5">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="0" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="1.073" y2="0" width="0.127" layer="21"/>
<wire x1="1.073" y1="0" x2="1.524" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-1.077" y2="0" width="0.127" layer="21"/>
<wire x1="-1.077" y1="0" x2="-1.524" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-3.275" y1="0" x2="-1.077" y2="0" width="0.127" layer="21"/>
<wire x1="1.073" y1="0" x2="3.275" y2="0" width="0.127" layer="21"/>
<wire x1="-3.132" y1="-1.089" x2="-2.194" y2="-1.089" width="0.127" layer="21"/>
<wire x1="-2.663" y1="-0.62" x2="-2.663" y2="-1.558" width="0.127" layer="21"/>
<circle x="0" y="0" radius="8.1339" width="0.127" layer="21"/>
<circle x="-3.816" y="0" radius="0.5732" width="0.127" layer="102"/>
<circle x="3.816" y="0" radius="0.5732" width="0.127" layer="102"/>
<pad name="C-" x="3.81" y="0" drill="1.016" diameter="2.54" shape="square"/>
<pad name="C+" x="-3.81" y="0" drill="1.016" diameter="2.54"/>
<text x="-4.458" y="-3.981" size="1.778" layer="27">&gt;VALUE</text>
<text x="-3.812" y="2.076" size="1.778" layer="25">&gt;NAME</text>
<text x="1.113" y="0.159" size="0.254" layer="100">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL7,5+">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="0" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="1.073" y2="0" width="0.127" layer="21"/>
<wire x1="1.073" y1="0" x2="1.524" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-1.077" y2="0" width="0.127" layer="21"/>
<wire x1="-1.077" y1="0" x2="-1.524" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-3.275" y1="0" x2="-1.077" y2="0" width="0.127" layer="21"/>
<wire x1="1.073" y1="0" x2="3.275" y2="0" width="0.127" layer="21"/>
<wire x1="-3.132" y1="-1.089" x2="-2.194" y2="-1.089" width="0.127" layer="21"/>
<wire x1="-2.663" y1="-0.62" x2="-2.663" y2="-1.558" width="0.127" layer="21"/>
<circle x="0" y="0" radius="9.063" width="0.127" layer="21"/>
<circle x="-3.816" y="0" radius="0.5732" width="0.127" layer="102"/>
<circle x="3.816" y="0" radius="0.5732" width="0.127" layer="102"/>
<pad name="C-" x="3.81" y="0" drill="1.016" diameter="2.54" shape="square"/>
<pad name="C+" x="-3.81" y="0" drill="1.016" diameter="2.54"/>
<text x="-4.458" y="-3.981" size="1.778" layer="27">&gt;VALUE</text>
<text x="-3.812" y="2.076" size="1.778" layer="25">&gt;NAME</text>
<text x="1.113" y="0.159" size="0.254" layer="100">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL_10">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="4.2863" y2="0" width="0.127" layer="21"/>
<wire x1="-0.7938" y1="0" x2="-4.2863" y2="0" width="0.127" layer="21"/>
<wire x1="-3.3338" y1="-1.1113" x2="-2.0638" y2="-1.1113" width="0.127" layer="21"/>
<wire x1="-2.6988" y1="-0.4763" x2="-2.6988" y2="-1.7463" width="0.127" layer="21"/>
<circle x="0" y="0" radius="11.1125" width="0.127" layer="21"/>
<circle x="-5.08" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.7099" width="0.127" layer="102"/>
<pad name="C-" x="5.08" y="0" drill="1.27" diameter="3.2" shape="square"/>
<pad name="C+" x="-5.08" y="0" drill="1.27" diameter="3.2"/>
<text x="-5.728" y="-4.9335" size="2.1844" layer="27">&gt;VALUE</text>
<text x="-4.7645" y="3.0285" size="2.1844" layer="25">&gt;NAME</text>
<text x="1.113" y="0.159" size="0.254" layer="100">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL_10+">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="4.2863" y2="0" width="0.127" layer="21"/>
<wire x1="-0.7938" y1="0" x2="-4.2863" y2="0" width="0.127" layer="21"/>
<wire x1="-3.3338" y1="-1.1113" x2="-2.0638" y2="-1.1113" width="0.127" layer="21"/>
<wire x1="-2.6988" y1="-0.4763" x2="-2.6988" y2="-1.7463" width="0.127" layer="21"/>
<circle x="-5.08" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="0" y="0" radius="15.24" width="0.254" layer="21"/>
<pad name="C-" x="5.08" y="0" drill="1.27" diameter="3.2" shape="square"/>
<pad name="C+" x="-5.08" y="0" drill="1.27" diameter="3.2"/>
<text x="-5.728" y="-5.886" size="2.1844" layer="27">&gt;VALUE</text>
<text x="-4.7645" y="3.0285" size="2.1844" layer="25">&gt;NAME</text>
<text x="1.113" y="0.159" size="0.254" layer="100">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="FED_2200">
<description>&lt;B&gt;Tlumivka FED&lt;/B&gt; - 2,2 mH &lt;I&gt;(pro stmivace)&lt;/I&gt;&lt;BR&gt;
2.3A, 340mOhm, max 500W  &lt;I&gt;(více na GES)&lt;/I&gt;</description>
<wire x1="-6.985" y1="0" x2="-12.573" y2="-1.27" width="1.016" layer="51"/>
<wire x1="-6.731" y1="-1.27" x2="-12.065" y2="-3.429" width="1.016" layer="51"/>
<wire x1="-6.858" y1="1.143" x2="-12.446" y2="1.524" width="1.016" layer="51"/>
<wire x1="-6.477" y1="-2.54" x2="-11.176" y2="-5.588" width="1.016" layer="21"/>
<wire x1="-5.969" y1="-3.556" x2="-10.287" y2="-7.112" width="1.016" layer="21"/>
<wire x1="-5.334" y1="-4.445" x2="-9.271" y2="-8.382" width="1.016" layer="21"/>
<wire x1="-4.572" y1="-5.334" x2="-8.128" y2="-9.525" width="1.016" layer="21"/>
<wire x1="-3.683" y1="-5.842" x2="-6.858" y2="-10.414" width="1.016" layer="21"/>
<wire x1="-2.794" y1="-6.477" x2="-5.461" y2="-11.303" width="1.016" layer="21"/>
<wire x1="-1.778" y1="-6.731" x2="-3.81" y2="-11.938" width="1.016" layer="21"/>
<wire x1="-0.762" y1="-7.112" x2="-1.651" y2="-12.319" width="1.016" layer="21"/>
<wire x1="0.381" y1="-7.112" x2="0.381" y2="-12.446" width="1.016" layer="21"/>
<wire x1="1.397" y1="-6.858" x2="2.413" y2="-12.192" width="1.016" layer="21"/>
<wire x1="2.54" y1="-6.604" x2="4.572" y2="-11.684" width="1.016" layer="21"/>
<wire x1="3.556" y1="-6.096" x2="6.223" y2="-10.795" width="1.016" layer="21"/>
<wire x1="4.572" y1="-5.334" x2="8.001" y2="-9.525" width="1.016" layer="21"/>
<wire x1="5.461" y1="-4.572" x2="9.398" y2="-8.128" width="1.016" layer="21"/>
<wire x1="6.096" y1="-3.556" x2="10.795" y2="-6.35" width="1.016" layer="21"/>
<wire x1="6.604" y1="-2.54" x2="11.684" y2="-4.699" width="1.016" layer="21"/>
<wire x1="6.858" y1="-1.397" x2="12.319" y2="-2.413" width="1.016" layer="21"/>
<wire x1="6.985" y1="-0.381" x2="12.573" y2="-0.381" width="1.016" layer="21"/>
<wire x1="6.985" y1="0.635" x2="12.446" y2="1.524" width="1.016" layer="21"/>
<wire x1="6.858" y1="1.651" x2="12.065" y2="3.302" width="1.016" layer="21"/>
<wire x1="6.477" y1="2.667" x2="11.557" y2="4.953" width="1.016" layer="21"/>
<wire x1="5.969" y1="3.683" x2="10.541" y2="6.731" width="1.016" layer="21"/>
<wire x1="5.334" y1="4.572" x2="9.398" y2="8.255" width="1.016" layer="21"/>
<wire x1="4.572" y1="5.207" x2="8.001" y2="9.525" width="1.016" layer="21"/>
<wire x1="3.683" y1="5.969" x2="6.604" y2="10.541" width="1.016" layer="21"/>
<wire x1="2.667" y1="6.477" x2="5.08" y2="11.43" width="1.016" layer="21"/>
<wire x1="1.524" y1="6.858" x2="2.921" y2="12.192" width="1.016" layer="21"/>
<wire x1="0.381" y1="6.985" x2="0.635" y2="12.573" width="1.016" layer="21"/>
<wire x1="-0.762" y1="6.985" x2="-1.524" y2="12.319" width="1.016" layer="21"/>
<wire x1="-1.778" y1="6.731" x2="-3.556" y2="11.938" width="1.016" layer="21"/>
<wire x1="-2.921" y1="6.35" x2="-5.588" y2="11.176" width="1.016" layer="21"/>
<wire x1="-3.937" y1="5.842" x2="-7.239" y2="10.16" width="1.016" layer="21"/>
<wire x1="-4.826" y1="5.08" x2="-9.017" y2="8.763" width="1.016" layer="21"/>
<wire x1="-5.588" y1="4.064" x2="-10.414" y2="7.112" width="1.016" layer="21"/>
<wire x1="-6.223" y1="3.175" x2="-11.303" y2="5.461" width="1.016" layer="21"/>
<wire x1="-6.731" y1="2.286" x2="-12.065" y2="3.556" width="1.016" layer="21"/>
<wire x1="-7.3819" y1="-1.8901" x2="-7.3819" y2="1.8901" width="0.127" layer="21" curve="331.276094" cap="flat"/>
<wire x1="-11.9146" y1="-1.8991" x2="-11.9146" y2="1.8991" width="0.127" layer="21" curve="341.88701" cap="flat"/>
<wire x1="-11.9146" y1="1.8991" x2="-11.9146" y2="-1.8991" width="0.127" layer="51" curve="18.112691" cap="flat"/>
<wire x1="-7.3819" y1="1.8901" x2="-7.3819" y2="-1.8901" width="0.127" layer="51" curve="28.723533" cap="flat"/>
<circle x="-13.335" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="-6.35" y="0" radius="0.7099" width="0.127" layer="102"/>
<pad name="1" x="-13.335" y="0" drill="1.27" diameter="3.2" shape="octagon"/>
<pad name="2" x="-6.35" y="0" drill="1.27" diameter="3.2" shape="octagon"/>
<text x="-2.2225" y="4.445" size="1.016" layer="101">DÝ1,0</text>
<text x="-3.4925" y="0.9525" size="1.778" layer="25">&gt;Name</text>
<text x="-4.1275" y="-3.175" size="1.778" layer="27">&gt;Value</text>
</package>
<package name="TO-220AC">
<wire x1="-5.0006" y1="1.3494" x2="-4.9213" y2="-1.905" width="0.127" layer="21"/>
<wire x1="5.0006" y1="1.3494" x2="4.9212" y2="-1.905" width="0.127" layer="21"/>
<wire x1="-4.9213" y1="-1.905" x2="4.9213" y2="-1.905" width="0.127" layer="21"/>
<circle x="-2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="A" x="-2.54" y="0" drill="1.016" diameter="2.54"/>
<pad name="K" x="2.54" y="0" drill="1.016" diameter="2.54" shape="square"/>
<text x="-4.929" y="2.703" size="1.27" layer="25">&gt;Name</text>
<text x="-0.795" y="2.703" size="1.27" layer="27">&gt;Value</text>
<text x="-4.77" y="-1.749" size="0.254" layer="100">PaJa</text>
<rectangle x1="-5.084" y1="1.27" x2="5.076" y2="2.54" layer="21"/>
</package>
<package name="TO-220S">
<wire x1="-5.0006" y1="1.3494" x2="-4.9213" y2="-1.905" width="0.127" layer="21"/>
<wire x1="5.0006" y1="1.3494" x2="4.9212" y2="-1.905" width="0.127" layer="21"/>
<wire x1="-4.9213" y1="-1.905" x2="4.9213" y2="-1.905" width="0.127" layer="21"/>
<circle x="-2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="0" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="1" x="-2.54" y="0" drill="1.016" diameter="1.6" shape="long" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.016" diameter="1.6" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" diameter="1.6" shape="long" rot="R90"/>
<text x="-4.929" y="2.703" size="1.27" layer="25">&gt;Name</text>
<text x="-0.795" y="2.703" size="1.27" layer="27">&gt;Value</text>
<text x="-4.77" y="-1.749" size="0.254" layer="100">PaJa</text>
<rectangle x1="-5.084" y1="1.27" x2="5.076" y2="2.54" layer="21"/>
</package>
<package name="TO-220">
<wire x1="-2.8575" y1="0.635" x2="-2.8575" y2="1.5875" width="0.127" layer="21"/>
<wire x1="-2.8575" y1="1.5875" x2="-3.175" y2="1.5875" width="0.127" layer="21"/>
<wire x1="-3.175" y1="1.5875" x2="-3.175" y2="5.08" width="0.127" layer="21"/>
<wire x1="-2.2225" y1="0.635" x2="-2.2225" y2="1.5875" width="0.127" layer="21"/>
<wire x1="-2.2225" y1="1.5875" x2="-1.905" y2="1.5875" width="0.127" layer="21"/>
<wire x1="-1.905" y1="1.5875" x2="-1.905" y2="5.08" width="0.127" layer="21"/>
<wire x1="-0.3175" y1="0.635" x2="-0.3175" y2="1.5875" width="0.127" layer="21"/>
<wire x1="-0.3175" y1="1.5875" x2="-0.635" y2="1.5875" width="0.127" layer="21"/>
<wire x1="-0.635" y1="1.5875" x2="-0.635" y2="5.08" width="0.127" layer="21"/>
<wire x1="0.635" y1="5.08" x2="0.635" y2="1.5875" width="0.127" layer="21"/>
<wire x1="0.635" y1="1.5875" x2="0.3175" y2="1.5875" width="0.127" layer="21"/>
<wire x1="0.3175" y1="1.5875" x2="0.3175" y2="0.635" width="0.127" layer="21"/>
<wire x1="1.905" y1="5.08" x2="1.905" y2="1.5875" width="0.127" layer="21"/>
<wire x1="1.905" y1="1.5875" x2="2.2225" y2="1.5875" width="0.127" layer="21"/>
<wire x1="2.2225" y1="1.5875" x2="2.2225" y2="0.635" width="0.127" layer="21"/>
<wire x1="2.8575" y1="0.635" x2="2.8575" y2="1.5875" width="0.127" layer="21"/>
<wire x1="2.8575" y1="1.5875" x2="3.175" y2="1.5875" width="0.127" layer="21"/>
<wire x1="3.175" y1="1.5875" x2="3.175" y2="5.08" width="0.127" layer="21"/>
<wire x1="-3.81" y1="20.6375" x2="3.81" y2="20.6375" width="0.254" layer="21"/>
<wire x1="-3.81" y1="20.6375" x2="-5.08" y2="18.415" width="0.254" layer="21"/>
<wire x1="3.81" y1="20.6375" x2="5.08" y2="18.415" width="0.254" layer="21"/>
<wire x1="-5.08" y1="18.415" x2="-5.08" y2="14.605" width="0.254" layer="21"/>
<wire x1="-5.08" y1="5.08" x2="5.08" y2="5.08" width="0.254" layer="21"/>
<wire x1="5.08" y1="18.415" x2="5.08" y2="14.605" width="0.254" layer="21"/>
<wire x1="5.08" y1="14.605" x2="5.08" y2="5.08" width="0.254" layer="21"/>
<wire x1="-5.08" y1="14.605" x2="5.08" y2="14.605" width="0.127" layer="21"/>
<wire x1="-4.7625" y1="13.6525" x2="4.7625" y2="13.6525" width="0.127" layer="51"/>
<wire x1="4.7625" y1="13.6525" x2="5.08" y2="14.605" width="0.127" layer="51"/>
<wire x1="-4.7625" y1="13.6525" x2="-5.08" y2="14.605" width="0.127" layer="51"/>
<wire x1="-4.7625" y1="7.3025" x2="-4.7625" y2="9.8425" width="0.127" layer="51" curve="180"/>
<wire x1="-4.7625" y1="13.6525" x2="-4.7625" y2="9.8425" width="0.127" layer="51"/>
<wire x1="-4.7625" y1="7.3025" x2="-4.7625" y2="5.3975" width="0.127" layer="51"/>
<wire x1="-4.7625" y1="5.3975" x2="4.7625" y2="5.3975" width="0.127" layer="51"/>
<wire x1="4.7625" y1="5.3975" x2="4.7625" y2="7.3025" width="0.127" layer="51"/>
<wire x1="4.7625" y1="13.6525" x2="4.7625" y2="9.8425" width="0.127" layer="51"/>
<wire x1="4.7625" y1="5.3975" x2="5.08" y2="5.08" width="0.127" layer="51"/>
<wire x1="-4.7625" y1="5.3975" x2="-5.08" y2="5.08" width="0.127" layer="51"/>
<wire x1="4.7625" y1="7.3025" x2="4.7625" y2="9.8425" width="0.127" layer="51" curve="-180"/>
<wire x1="-4.7625" y1="9.8425" x2="-5.08" y2="9.525" width="0.127" layer="51"/>
<wire x1="-4.7625" y1="9.525" x2="-4.7625" y2="7.62" width="0.127" layer="51" curve="-180"/>
<wire x1="-4.7625" y1="9.525" x2="-5.08" y2="9.525" width="0.127" layer="51"/>
<wire x1="-5.08" y1="7.62" x2="-4.7625" y2="7.62" width="0.127" layer="51"/>
<wire x1="-5.08" y1="7.62" x2="-4.7625" y2="7.3025" width="0.127" layer="51"/>
<wire x1="-5.08" y1="14.605" x2="-5.08" y2="5.08" width="0.254" layer="21"/>
<wire x1="4.7625" y1="9.8425" x2="5.08" y2="9.525" width="0.127" layer="51"/>
<wire x1="5.08" y1="7.62" x2="4.7625" y2="7.3025" width="0.127" layer="51"/>
<wire x1="5.08" y1="7.62" x2="4.7625" y2="7.62" width="0.127" layer="51"/>
<wire x1="4.7625" y1="9.525" x2="5.08" y2="9.525" width="0.127" layer="51"/>
<wire x1="4.7625" y1="9.525" x2="4.7625" y2="7.62" width="0.127" layer="51" curve="180"/>
<circle x="-2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="0" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.5723" width="0.127" layer="102"/>
<circle x="0" y="17.78" radius="1.7097" width="0.127" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="1.016" diameter="1.6" shape="long" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.016" diameter="1.6" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" diameter="1.6" shape="long" rot="R90"/>
<text x="-3.6525" y="10.815" size="1.6764" layer="25">&gt;NAME</text>
<text x="-3.971" y="5.7365" size="1.6764" layer="27">&gt;VALUE</text>
<text x="-0.489" y="14.1335" size="0.254" layer="100">PaJa</text>
<text x="-2.54" y="14.9225" size="1.016" layer="101">TO-220</text>
</package>
<package name="LED_10">
<description>&lt;B&gt;LED dioda&lt;/B&gt; - 10mm prumer</description>
<wire x1="-1.268" y1="-0.446" x2="-1.268" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="1.272" y1="-1.5875" x2="1.272" y2="-0.446" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-0.8255" x2="-0.633" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-2.3495" x2="0.637" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.5875" x2="1.272" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.5875" x2="-0.633" y2="-0.8255" width="0.127" layer="21"/>
<wire x1="-0.347" y1="-2.7303" x2="0.415" y2="-3.4923" width="0.127" layer="21"/>
<wire x1="0.288" y1="-2.3493" x2="1.05" y2="-3.1113" width="0.127" layer="21"/>
<wire x1="0.034" y1="-3.3653" x2="0.288" y2="-3.1113" width="0.127" layer="21"/>
<wire x1="0.288" y1="-3.1113" x2="0.415" y2="-3.4923" width="0.127" layer="21"/>
<wire x1="0.415" y1="-3.4923" x2="0.034" y2="-3.3653" width="0.127" layer="21"/>
<wire x1="1.05" y1="-3.1113" x2="0.669" y2="-2.9843" width="0.127" layer="21"/>
<wire x1="0.669" y1="-2.9843" x2="0.923" y2="-2.7303" width="0.127" layer="21"/>
<wire x1="0.923" y1="-2.7303" x2="1.05" y2="-3.1113" width="0.127" layer="21"/>
<wire x1="0.796" y1="-2.9843" x2="0.923" y2="-2.8573" width="0.127" layer="21"/>
<wire x1="0.161" y1="-3.3653" x2="0.288" y2="-3.2383" width="0.127" layer="21"/>
<wire x1="0.637" y1="-2.3495" x2="0.637" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.5875" x2="0.637" y2="-0.8255" width="0.127" layer="21"/>
<wire x1="-1.268" y1="-1.5875" x2="-0.633" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-1.5875" x2="-0.633" y2="-2.3495" width="0.127" layer="21"/>
<wire x1="3.81" y1="-2.2225" x2="3.81" y2="2.2225" width="0.127" layer="21"/>
<wire x1="3.81" y1="-2.2225" x2="3.81" y2="2.2225" width="0.127" layer="21" curve="-299.487126"/>
<circle x="-1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="0" y="0" radius="3.81" width="0.127" layer="51"/>
<pad name="K" x="1.27" y="0" drill="0.8128" diameter="1.9304" shape="square"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" diameter="1.9304"/>
<text x="-2.2268" y="1.4225" size="1.27" layer="25">&gt;Name</text>
<text x="5.2472" y="-2.8622" size="1.27" layer="27" rot="R90">&gt;Value</text>
<text x="0.793" y="-2.0675" size="0.254" layer="100">PaJa</text>
</package>
<package name="LED_3">
<description>&lt;B&gt;LED dioda&lt;/B&gt; - 3mm prumer</description>
<wire x1="-0.381" y1="-0.381" x2="-0.381" y2="-0.889" width="0.127" layer="21"/>
<wire x1="-0.381" y1="-1.397" x2="0.381" y2="-0.889" width="0.127" layer="21"/>
<wire x1="0.381" y1="-0.889" x2="0.889" y2="-0.889" width="0.127" layer="21"/>
<wire x1="0.381" y1="-0.889" x2="-0.381" y2="-0.381" width="0.127" layer="21"/>
<wire x1="-0.635" y1="0.508" x2="0.127" y2="1.27" width="0.127" layer="21"/>
<wire x1="0" y1="0.127" x2="0.762" y2="0.889" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.143" x2="0" y2="0.889" width="0.127" layer="21"/>
<wire x1="0" y1="0.889" x2="0.127" y2="1.27" width="0.127" layer="21"/>
<wire x1="0.127" y1="1.27" x2="-0.254" y2="1.143" width="0.127" layer="21"/>
<wire x1="0.762" y1="0.889" x2="0.381" y2="0.762" width="0.127" layer="21"/>
<wire x1="0.381" y1="0.762" x2="0.635" y2="0.508" width="0.127" layer="21"/>
<wire x1="0.635" y1="0.508" x2="0.762" y2="0.889" width="0.127" layer="21"/>
<wire x1="0.508" y1="0.762" x2="0.635" y2="0.635" width="0.127" layer="21"/>
<wire x1="-0.127" y1="1.143" x2="0" y2="1.016" width="0.127" layer="21"/>
<wire x1="0.381" y1="-1.397" x2="0.381" y2="-0.889" width="0.127" layer="21"/>
<wire x1="0.381" y1="-0.889" x2="0.381" y2="-0.381" width="0.127" layer="21"/>
<wire x1="0.889" y1="-0.889" x2="1.1685" y2="-0.5239" width="0.127" layer="21"/>
<wire x1="-0.8889" y1="-0.889" x2="-1.0921" y2="-0.5238" width="0.127" layer="21"/>
<wire x1="-0.8889" y1="-0.889" x2="-0.381" y2="-0.889" width="0.127" layer="21"/>
<wire x1="-0.381" y1="-0.889" x2="-0.381" y2="-1.397" width="0.127" layer="21"/>
<wire x1="-1.5081" y1="0.4763" x2="1.4289" y2="0.7145" width="0.127" layer="21" curve="-135.860035" cap="flat"/>
<wire x1="-1.5081" y1="-0.4763" x2="1.4288" y2="-0.7144" width="0.127" layer="21" curve="135.855325" cap="flat"/>
<wire x1="1.4288" y1="0.7144" x2="1.4288" y2="0.4763" width="0.127" layer="21"/>
<wire x1="1.4288" y1="-0.7144" x2="1.4288" y2="-0.4763" width="0.127" layer="21"/>
<wire x1="1.4288" y1="0.4763" x2="1.4288" y2="-0.4763" width="0.127" layer="51"/>
<wire x1="-1.5081" y1="0.4763" x2="-1.5081" y2="-0.4763" width="0.127" layer="51" curve="35.055137" cap="flat"/>
<circle x="-1.272" y="0" radius="0.5028" width="0.127" layer="102"/>
<circle x="1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" diameter="1.778"/>
<pad name="K" x="1.27" y="0" drill="0.8128" diameter="1.778" shape="square"/>
<text x="-2.544" y="1.749" size="1.27" layer="25">&gt;Name</text>
<text x="-2.703" y="-3.021" size="1.27" layer="27">&gt;Value</text>
<text x="-0.318" y="0.636" size="0.254" layer="100" rot="R270">PaJa</text>
</package>
<package name="LED_5">
<description>&lt;B&gt;LED dioda&lt;/B&gt; - 5mm prumer</description>
<wire x1="-1.268" y1="-0.446" x2="-1.268" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.272" y1="-1.27" x2="1.272" y2="-0.446" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-0.508" x2="-0.633" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-2.032" x2="0.637" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.27" x2="1.272" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.27" x2="-0.633" y2="-0.508" width="0.127" layer="21"/>
<wire x1="-0.347" y1="0.984" x2="0.415" y2="1.746" width="0.127" layer="21"/>
<wire x1="0.288" y1="0.603" x2="1.05" y2="1.365" width="0.127" layer="21"/>
<wire x1="0.034" y1="1.619" x2="0.288" y2="1.365" width="0.127" layer="21"/>
<wire x1="0.288" y1="1.365" x2="0.415" y2="1.746" width="0.127" layer="21"/>
<wire x1="0.415" y1="1.746" x2="0.034" y2="1.619" width="0.127" layer="21"/>
<wire x1="1.05" y1="1.365" x2="0.669" y2="1.238" width="0.127" layer="21"/>
<wire x1="0.669" y1="1.238" x2="0.923" y2="0.984" width="0.127" layer="21"/>
<wire x1="0.923" y1="0.984" x2="1.05" y2="1.365" width="0.127" layer="21"/>
<wire x1="0.796" y1="1.238" x2="0.923" y2="1.111" width="0.127" layer="21"/>
<wire x1="0.161" y1="1.619" x2="0.288" y2="1.492" width="0.127" layer="21"/>
<wire x1="0.637" y1="-2.032" x2="0.637" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.27" x2="0.637" y2="-0.508" width="0.127" layer="21"/>
<wire x1="-1.268" y1="-1.27" x2="-0.633" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-1.27" x2="-0.633" y2="-2.032" width="0.127" layer="21"/>
<wire x1="2.544" y1="-1.431" x2="2.544" y2="1.431" width="0.127" layer="21" curve="-301.284493"/>
<wire x1="2.544" y1="1.431" x2="2.544" y2="-1.431" width="0.127" layer="21"/>
<circle x="-1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="0" y="0" radius="2.5489" width="0.127" layer="51"/>
<pad name="K" x="1.27" y="0" drill="0.8128" diameter="1.778" shape="square"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" diameter="1.778"/>
<text x="3.975" y="-2.703" size="1.27" layer="25" rot="R90">&gt;Name</text>
<text x="5.7235" y="-2.7035" size="1.27" layer="27" rot="R90">&gt;Value</text>
<text x="-0.477" y="-2.385" size="0.254" layer="100">PaJa</text>
</package>
<package name="LED_5X5">
<description>&lt;B&gt;LED dioda&lt;/B&gt; - ctverec - 5mm x 5mm</description>
<wire x1="-1.27" y1="0.031" x2="-1.27" y2="-0.9515" width="0.127" layer="21"/>
<wire x1="1.27" y1="-0.9515" x2="1.27" y2="0.031" width="0.127" layer="21"/>
<wire x1="-0.635" y1="-0.1895" x2="-0.635" y2="-0.9515" width="0.127" layer="21"/>
<wire x1="-0.635" y1="-1.7135" x2="0.635" y2="-0.9515" width="0.127" layer="21"/>
<wire x1="0.635" y1="-0.9515" x2="1.27" y2="-0.9515" width="0.127" layer="21"/>
<wire x1="0.635" y1="-0.9515" x2="-0.635" y2="-0.1895" width="0.127" layer="21"/>
<wire x1="-0.349" y1="0.3485" x2="0.413" y2="1.1105" width="0.127" layer="21"/>
<wire x1="0.286" y1="-0.0325" x2="1.048" y2="0.7295" width="0.127" layer="21"/>
<wire x1="0.032" y1="0.9835" x2="0.286" y2="0.7295" width="0.127" layer="21"/>
<wire x1="0.286" y1="0.7295" x2="0.413" y2="1.1105" width="0.127" layer="21"/>
<wire x1="0.413" y1="1.1105" x2="0.032" y2="0.9835" width="0.127" layer="21"/>
<wire x1="1.048" y1="0.7295" x2="0.667" y2="0.6025" width="0.127" layer="21"/>
<wire x1="0.667" y1="0.6025" x2="0.921" y2="0.3485" width="0.127" layer="21"/>
<wire x1="0.921" y1="0.3485" x2="1.048" y2="0.7295" width="0.127" layer="21"/>
<wire x1="0.794" y1="0.6025" x2="0.921" y2="0.4755" width="0.127" layer="21"/>
<wire x1="0.159" y1="0.9835" x2="0.286" y2="0.8565" width="0.127" layer="21"/>
<wire x1="0.635" y1="-1.7135" x2="0.635" y2="-0.9515" width="0.127" layer="21"/>
<wire x1="0.635" y1="-0.9515" x2="0.635" y2="-0.1895" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-0.9515" x2="-0.635" y2="-0.9515" width="0.127" layer="21"/>
<wire x1="-0.635" y1="-0.9515" x2="-0.635" y2="-1.7135" width="0.127" layer="21"/>
<wire x1="2.5399" y1="-2.54" x2="-2.54" y2="-2.54" width="0.127" layer="21"/>
<wire x1="2.5399" y1="2.54" x2="2.5399" y2="-2.54" width="0.127" layer="21"/>
<wire x1="2.5399" y1="2.54" x2="-2.54" y2="2.54" width="0.127" layer="21"/>
<wire x1="-2.54" y1="2.54" x2="-2.54" y2="-2.54" width="0.127" layer="21"/>
<circle x="-1.274" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="1.27" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="K" x="1.27" y="0" drill="0.8128" diameter="1.9304" shape="square"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" diameter="1.9304"/>
<text x="-2.228" y="1.2722" size="1.016" layer="25">&gt;Name</text>
<text x="-2.387" y="-3.655" size="1.016" layer="27">&gt;Value</text>
<text x="1.27" y="-2.3795" size="0.254" layer="100">PaJa</text>
</package>
<package name="LED_8">
<description>&lt;B&gt;LED dioda&lt;/B&gt; - 8mm prumer</description>
<wire x1="-1.268" y1="-0.446" x2="-1.268" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="1.272" y1="-1.5875" x2="1.272" y2="-0.446" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-0.8255" x2="-0.633" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-2.3495" x2="0.637" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.5875" x2="1.272" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.5875" x2="-0.633" y2="-0.8255" width="0.127" layer="21"/>
<wire x1="-0.347" y1="-2.7303" x2="0.415" y2="-3.4923" width="0.127" layer="21"/>
<wire x1="0.288" y1="-2.3493" x2="1.05" y2="-3.1113" width="0.127" layer="21"/>
<wire x1="0.034" y1="-3.3653" x2="0.288" y2="-3.1113" width="0.127" layer="21"/>
<wire x1="0.288" y1="-3.1113" x2="0.415" y2="-3.4923" width="0.127" layer="21"/>
<wire x1="0.415" y1="-3.4923" x2="0.034" y2="-3.3653" width="0.127" layer="21"/>
<wire x1="1.05" y1="-3.1113" x2="0.669" y2="-2.9843" width="0.127" layer="21"/>
<wire x1="0.669" y1="-2.9843" x2="0.923" y2="-2.7303" width="0.127" layer="21"/>
<wire x1="0.923" y1="-2.7303" x2="1.05" y2="-3.1113" width="0.127" layer="21"/>
<wire x1="0.796" y1="-2.9843" x2="0.923" y2="-2.8573" width="0.127" layer="22"/>
<wire x1="0.161" y1="-3.3653" x2="0.288" y2="-3.2383" width="0.127" layer="21"/>
<wire x1="0.637" y1="-2.3495" x2="0.637" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="0.637" y1="-1.5875" x2="0.637" y2="-0.8255" width="0.127" layer="21"/>
<wire x1="-1.268" y1="-1.5875" x2="-0.633" y2="-1.5875" width="0.127" layer="21"/>
<wire x1="-0.633" y1="-1.5875" x2="-0.633" y2="-2.3495" width="0.127" layer="21"/>
<wire x1="3.81" y1="-2.2225" x2="3.81" y2="2.2225" width="0.127" layer="21"/>
<wire x1="3.81" y1="-2.2225" x2="3.81" y2="2.2225" width="0.127" layer="21" curve="-299.487126"/>
<circle x="-1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="0" y="0" radius="3.81" width="0.127" layer="51"/>
<pad name="K" x="1.27" y="0" drill="0.8128" diameter="1.9304" shape="square"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" diameter="1.9304"/>
<text x="-2.2268" y="1.4225" size="1.27" layer="25">&gt;Name</text>
<text x="5.2472" y="-2.386" size="1.27" layer="27" rot="R90">&gt;Value</text>
<text x="0.793" y="-2.0675" size="0.254" layer="100">PaJa</text>
</package>
<package name="LED_TROJ">
<description>&lt;B&gt;LED dioda&lt;/B&gt; - trojuhelnik</description>
<wire x1="-0.637" y1="-0.764" x2="-0.637" y2="-0.002" width="0.127" layer="21"/>
<wire x1="-1.907" y1="-0.002" x2="-2.542" y2="-0.002" width="0.127" layer="21"/>
<wire x1="-0.822" y1="1.143" x2="-0.06" y2="1.905" width="0.127" layer="21"/>
<wire x1="-0.187" y1="0.762" x2="0.575" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.441" y1="1.778" x2="-0.187" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.187" y1="1.524" x2="-0.06" y2="1.905" width="0.127" layer="21"/>
<wire x1="-0.06" y1="1.905" x2="-0.441" y2="1.778" width="0.127" layer="21"/>
<wire x1="0.575" y1="1.524" x2="0.194" y2="1.397" width="0.127" layer="21"/>
<wire x1="0.194" y1="1.397" x2="0.448" y2="1.143" width="0.127" layer="21"/>
<wire x1="0.448" y1="1.143" x2="0.575" y2="1.524" width="0.127" layer="21"/>
<wire x1="0.321" y1="1.397" x2="0.448" y2="1.27" width="0.127" layer="21"/>
<wire x1="-0.314" y1="1.778" x2="-0.187" y2="1.651" width="0.127" layer="21"/>
<wire x1="-1.907" y1="0.76" x2="-1.907" y2="-0.002" width="0.127" layer="21"/>
<wire x1="-1.907" y1="-0.002" x2="-1.907" y2="-0.764" width="0.127" layer="21"/>
<wire x1="-0.002" y1="-0.002" x2="-0.637" y2="-0.002" width="0.127" layer="21"/>
<wire x1="-0.637" y1="-0.002" x2="-0.637" y2="0.76" width="0.127" layer="21"/>
<wire x1="-1.903" y1="-0.76" x2="-0.633" y2="0.002" width="0.127" layer="21"/>
<wire x1="-0.633" y1="0.002" x2="-1.903" y2="0.764" width="0.127" layer="21"/>
<wire x1="0.9525" y1="-2.6988" x2="0.9525" y2="2.6988" width="0.127" layer="21"/>
<wire x1="0.9525" y1="2.6988" x2="-3.81" y2="0" width="0.127" layer="21"/>
<wire x1="-3.81" y1="0" x2="0.9525" y2="-2.6988" width="0.127" layer="21"/>
<circle x="0.002" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="-2.542" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="K" x="0" y="0" drill="0.8128" diameter="1.9304" shape="square" rot="R180"/>
<pad name="A" x="-2.54" y="0" drill="0.8128" diameter="1.9304" rot="R180"/>
<text x="1.2737" y="2.226" size="1.016" layer="25" rot="R270">&gt;Name</text>
<text x="2.3868" y="2.544" size="1.016" layer="27" rot="R270">&gt;Value</text>
<text x="0.7964" y="-1.113" size="0.254" layer="100" rot="R180">PaJa</text>
</package>
<package name="LED2,5X5">
<description>&lt;B&gt;LED dioda&lt;/B&gt; - obdelnik - 5mm x 2,5mm</description>
<wire x1="0.639" y1="0.287" x2="0.639" y2="-0.475" width="0.127" layer="21"/>
<wire x1="0.639" y1="-1.237" x2="1.909" y2="-0.475" width="0.127" layer="21"/>
<wire x1="1.909" y1="-0.475" x2="2.544" y2="-0.475" width="0.127" layer="21"/>
<wire x1="1.909" y1="-0.475" x2="0.639" y2="0.287" width="0.127" layer="21"/>
<wire x1="1.561" y1="0.348" x2="2.323" y2="1.11" width="0.127" layer="21"/>
<wire x1="2.196" y1="-0.033" x2="2.958" y2="0.729" width="0.127" layer="21"/>
<wire x1="1.942" y1="0.983" x2="2.196" y2="0.729" width="0.127" layer="21"/>
<wire x1="2.196" y1="0.729" x2="2.323" y2="1.11" width="0.127" layer="21"/>
<wire x1="2.323" y1="1.11" x2="1.942" y2="0.983" width="0.127" layer="21"/>
<wire x1="2.958" y1="0.729" x2="2.577" y2="0.602" width="0.127" layer="21"/>
<wire x1="2.577" y1="0.602" x2="2.831" y2="0.348" width="0.127" layer="21"/>
<wire x1="2.831" y1="0.348" x2="2.958" y2="0.729" width="0.127" layer="21"/>
<wire x1="2.704" y1="0.602" x2="2.831" y2="0.475" width="0.127" layer="21"/>
<wire x1="2.069" y1="0.983" x2="2.196" y2="0.856" width="0.127" layer="21"/>
<wire x1="1.909" y1="-1.237" x2="1.909" y2="-0.475" width="0.127" layer="21"/>
<wire x1="1.909" y1="-0.475" x2="1.909" y2="0.287" width="0.127" layer="21"/>
<wire x1="0.004" y1="0.002" x2="0" y2="0.002" width="0.127" layer="21"/>
<wire x1="0" y1="0.002" x2="0" y2="-0.477" width="0.127" layer="21"/>
<wire x1="0" y1="-0.477" x2="0.639" y2="-0.475" width="0.127" layer="21"/>
<wire x1="0.639" y1="-0.475" x2="0.639" y2="-1.237" width="0.127" layer="21"/>
<wire x1="2.544" y1="-0.477" x2="2.544" y2="0" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="3.81" y2="1.27" width="0.127" layer="21"/>
<wire x1="3.81" y1="1.27" x2="3.81" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.81" y1="-1.27" x2="-1.27" y2="-1.27" width="0.127" layer="21"/>
<circle x="0" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="K" x="2.54" y="0" drill="0.8128" diameter="1.9304" shape="square"/>
<pad name="A" x="0" y="0" drill="0.8128" diameter="1.9304"/>
<text x="-1.113" y="1.431" size="1.016" layer="25">&gt;Name</text>
<text x="-1.431" y="-2.385" size="1.016" layer="27">&gt;Value</text>
<text x="1.431" y="0.159" size="0.254" layer="100" rot="R90">PaJa</text>
</package>
<package name="P1206">
<wire x1="-1.7463" y1="0.7938" x2="-1.7463" y2="0.3176" width="0.127" layer="21"/>
<wire x1="-1.1113" y1="-0.7937" x2="1.4287" y2="-0.7937" width="0.127" layer="21"/>
<wire x1="1.4287" y1="0.7938" x2="0.7937" y2="0.7938" width="0.127" layer="21"/>
<wire x1="0.7937" y1="0.7938" x2="-1.1113" y2="0.7938" width="0.127" layer="21"/>
<wire x1="-1.1113" y1="0.7938" x2="-1.7463" y2="0.7938" width="0.127" layer="21"/>
<wire x1="-1.1113" y1="0.7938" x2="-1.1113" y2="-0.7937" width="0.127" layer="21"/>
<wire x1="1.4288" y1="0.3175" x2="1.4288" y2="-0.3175" width="0.127" layer="21" curve="180"/>
<wire x1="-1.7462" y1="0.3175" x2="-1.7462" y2="-0.3175" width="0.127" layer="21" curve="-180" cap="flat"/>
<wire x1="-1.7462" y1="-0.3175" x2="-1.7462" y2="-0.7938" width="0.127" layer="21"/>
<wire x1="-1.1112" y1="-0.7938" x2="-1.7462" y2="-0.7938" width="0.127" layer="21"/>
<wire x1="1.4288" y1="0.7937" x2="1.4288" y2="0.3175" width="0.127" layer="21"/>
<wire x1="1.4288" y1="-0.3175" x2="1.4288" y2="-0.7938" width="0.127" layer="21"/>
<wire x1="0.1588" y1="-0.4763" x2="0.1588" y2="0" width="0.127" layer="21"/>
<wire x1="0.1588" y1="0" x2="0.1588" y2="0.4763" width="0.127" layer="21"/>
<wire x1="0.1588" y1="0" x2="-0.4763" y2="-0.4763" width="0.127" layer="21"/>
<wire x1="-0.4763" y1="-0.4763" x2="-0.4761" y2="0" width="0.127" layer="21"/>
<wire x1="-0.4761" y1="0" x2="-0.4763" y2="0.4762" width="0.127" layer="21"/>
<wire x1="-0.4763" y1="0.4762" x2="0.1588" y2="0" width="0.127" layer="21"/>
<wire x1="0.1588" y1="0" x2="0.4763" y2="0" width="0.127" layer="21"/>
<wire x1="-0.4761" y1="0" x2="-0.7937" y2="0" width="0.127" layer="21"/>
<wire x1="-1.7463" y1="-0.3175" x2="-1.7463" y2="0.3175" width="0.127" layer="51" curve="180" cap="flat"/>
<wire x1="0.7938" y1="0.7938" x2="1.4288" y2="0.7938" width="0.127" layer="51"/>
<wire x1="1.4288" y1="-0.3175" x2="1.4288" y2="0.3175" width="0.127" layer="51" curve="-180" cap="flat"/>
<wire x1="0.7938" y1="0.7938" x2="0.7938" y2="0.3175" width="0.127" layer="51"/>
<wire x1="0.7937" y1="0.7938" x2="0.7937" y2="-0.7937" width="0.127" layer="21"/>
<circle x="1.0319" y="0.5556" radius="0.2024" width="0.127" layer="51"/>
<smd name="K" x="1.5875" y="0" dx="0.9144" dy="1.778" layer="1" rot="R180"/>
<smd name="A" x="-1.9051" y="0.0001" dx="0.9144" dy="1.778" layer="1" rot="R180"/>
<text x="-0.9922" y="0.4762" size="0.254" layer="100" rot="R270">PaJa</text>
<text x="1.1113" y="-1.1113" size="1.016" layer="25" rot="R180">&gt;Name</text>
<text x="1.1113" y="2.0638" size="1.016" layer="27" rot="R180">&gt;Value</text>
<rectangle x1="-0.3174" y1="-0.1588" x2="0" y2="0.1588" layer="51" rot="R180"/>
<polygon width="0.127" layer="51">
<vertex x="-1.7463" y="-0.3175"/>
<vertex x="-1.7463" y="-0.7938"/>
<vertex x="-1.1113" y="-0.7938"/>
<vertex x="-1.1113" y="0.7938"/>
<vertex x="-1.7463" y="0.7938"/>
<vertex x="-1.7463" y="0.3175"/>
<vertex x="-1.5875" y="0.3175"/>
<vertex x="-1.4288" y="0.1588"/>
<vertex x="-1.4288" y="-0.1588"/>
<vertex x="-1.5875" y="-0.3175"/>
</polygon>
<polygon width="0.127" layer="51">
<vertex x="1.4288" y="0.7938"/>
<vertex x="1.4288" y="0.3174"/>
<vertex x="1.4288" y="0.3175"/>
<vertex x="1.27" y="0.3175"/>
<vertex x="1.1113" y="0.1588"/>
<vertex x="1.1113" y="-0.1588"/>
<vertex x="1.27" y="-0.3175"/>
<vertex x="1.4288" y="-0.3175"/>
<vertex x="1.4288" y="-0.7938"/>
<vertex x="0.7938" y="-0.7938"/>
<vertex x="0.7938" y="-0.635"/>
<vertex x="0.7937" y="-0.635"/>
<vertex x="0.7937" y="0.3175"/>
<vertex x="1.27" y="0.3175"/>
<vertex x="1.27" y="0.7938"/>
</polygon>
</package>
<package name="C-2,5">
<circle x="-1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<text x="-2.226" y="1.272" size="1.27" layer="25">&gt;Name</text>
<text x="-2.544" y="-2.544" size="1.27" layer="27">&gt;Value</text>
<text x="0.159" y="0.318" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-0.635" y1="-1.27" x2="-0.254" y2="1.27" layer="21"/>
<rectangle x1="0.254" y1="-1.27" x2="0.635" y2="1.27" layer="21"/>
<rectangle x1="-0.7938" y1="-0.1588" x2="-0.635" y2="0.1588" layer="21"/>
<rectangle x1="0.635" y1="-0.1588" x2="0.7938" y2="0.1588" layer="21"/>
</package>
<package name="C-5">
<circle x="-2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="0.795" y="0.954" size="1.016" layer="25">&gt;Name</text>
<text x="0.795" y="-1.9085" size="1.016" layer="27">&gt;Value</text>
<text x="0.159" y="0.3182" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-0.7143" y1="-1.27" x2="-0.238" y2="1.27" layer="21"/>
<rectangle x1="0.2381" y1="-1.27" x2="0.7144" y2="1.27" layer="21"/>
<rectangle x1="-2.0638" y1="-0.1588" x2="-0.635" y2="0.1588" layer="21"/>
<rectangle x1="0.635" y1="-0.1588" x2="2.0638" y2="0.1588" layer="21"/>
</package>
<package name="C-7,5">
<circle x="-3.814" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="3.814" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="0.795" y="0.954" size="1.016" layer="25">&gt;Name</text>
<text x="0.795" y="-1.9085" size="1.016" layer="27">&gt;Value</text>
<text x="0.159" y="0.477" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-0.7155" y1="-1.431" x2="-0.2385" y2="1.431" layer="21"/>
<rectangle x1="0.2385" y1="-1.431" x2="0.7155" y2="1.431" layer="21"/>
<rectangle x1="-3.3338" y1="-0.1588" x2="-0.635" y2="0.1588" layer="21"/>
<rectangle x1="0.635" y1="-0.1588" x2="3.3338" y2="0.1588" layer="21"/>
</package>
<package name="C-10">
<wire x1="-6.35" y1="2.6035" x2="-6.35" y2="-2.6035" width="0.127" layer="21"/>
<wire x1="-6.35" y1="-2.6035" x2="6.35" y2="-2.6035" width="0.127" layer="21"/>
<wire x1="6.35" y1="-2.6035" x2="6.35" y2="2.6035" width="0.127" layer="21"/>
<wire x1="6.35" y1="2.6035" x2="-6.35" y2="2.6035" width="0.127" layer="21"/>
<circle x="-5.08" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" diameter="2.1844" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" diameter="2.1844" shape="octagon"/>
<text x="0.159" y="0.3182" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="-4.0444" y="1.1525" size="1.27" layer="25">&gt;Name</text>
<text x="-4.3507" y="-2.4225" size="1.27" layer="27">&gt;Value</text>
<rectangle x1="-0.7144" y1="-1.27" x2="-0.2381" y2="1.27" layer="21"/>
<rectangle x1="0.238" y1="-1.27" x2="0.7143" y2="1.27" layer="21"/>
<rectangle x1="-4.6038" y1="-0.1588" x2="-0.635" y2="0.1588" layer="21"/>
<rectangle x1="0.635" y1="-0.1588" x2="4.6038" y2="0.1588" layer="21"/>
</package>
<package name="1206">
<description>&lt;B&gt;SMD&lt;/B&gt; - velikost 1206</description>
<wire x1="-1.0541" y1="0.7938" x2="1.0541" y2="0.7938" width="0.127" layer="21"/>
<wire x1="-1.0541" y1="-0.7938" x2="1.0541" y2="-0.7938" width="0.127" layer="21"/>
<wire x1="1.0541" y1="0.7938" x2="1.0541" y2="-0.7938" width="0.127" layer="21"/>
<wire x1="-1.0541" y1="0.7938" x2="-1.0541" y2="-0.7938" width="0.127" layer="21"/>
<smd name="1" x="-1.5875" y="0" dx="1.143" dy="1.7018" layer="1"/>
<smd name="2" x="1.5875" y="0" dx="1.143" dy="1.7018" layer="1"/>
<text x="-0.3175" y="-1.1906" size="0.254" layer="100">PaJa</text>
<text x="-0.7938" y="-0.4763" size="1.016" layer="25">&gt;Name</text>
<text x="-0.7938" y="0.9525" size="1.016" layer="27">&gt;Value</text>
<rectangle x1="-1.4541" y1="-0.7874" x2="-0.9461" y2="0.7874" layer="51"/>
<rectangle x1="0.9461" y1="-0.7874" x2="1.4541" y2="0.7874" layer="51"/>
</package>
<package name="0805">
<description>&lt;B&gt;SMD&lt;/B&gt; - velikost 0805</description>
<wire x1="0.5557" y1="0.5557" x2="-0.5557" y2="0.5557" width="0.127" layer="21"/>
<wire x1="-0.5557" y1="0.5557" x2="-0.5557" y2="-0.5556" width="0.127" layer="21"/>
<wire x1="-0.5557" y1="-0.5556" x2="0.5557" y2="-0.5556" width="0.127" layer="21"/>
<wire x1="0.5557" y1="-0.5556" x2="0.5557" y2="0.5557" width="0.127" layer="21"/>
<smd name="1" x="-0.9525" y="0" dx="1.016" dy="1.4224" layer="1"/>
<smd name="2" x="0.9525" y="0" dx="1.016" dy="1.4224" layer="1"/>
<text x="-1.397" y="-1.6351" size="0.8128" layer="27" ratio="10">&gt;VALUE</text>
<text x="-1.3177" y="0.8413" size="0.8128" layer="25" ratio="10">&gt;NAME</text>
<text x="0.4763" y="-0.4763" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="0.4064" y1="-0.6096" x2="0.9144" y2="0.6096" layer="51"/>
<rectangle x1="-0.9144" y1="-0.6096" x2="-0.4064" y2="0.6096" layer="51"/>
</package>
<package name="R-5">
<description>&lt;B&gt;Odpor&lt;/B&gt; - vel. 0204 - 0,4W - miniaturni</description>
<wire x1="-1.778" y1="0.635" x2="-1.524" y2="0.889" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-1.778" y1="-0.635" x2="-1.524" y2="-0.889" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="1.524" y1="-0.889" x2="1.778" y2="-0.635" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="1.524" y1="0.889" x2="1.778" y2="0.6388" width="0.127" layer="21" curve="-89.149199"/>
<wire x1="1.778" y1="0.6388" x2="1.778" y2="0.635" width="0.127" layer="21" curve="-0.857165"/>
<wire x1="-1.524" y1="0.889" x2="-1.27" y2="0.889" width="0.127" layer="21"/>
<wire x1="-1.143" y1="0.762" x2="-1.27" y2="0.889" width="0.127" layer="21"/>
<wire x1="-1.524" y1="-0.889" x2="-1.27" y2="-0.889" width="0.127" layer="21"/>
<wire x1="-1.143" y1="-0.762" x2="-1.27" y2="-0.889" width="0.127" layer="21"/>
<wire x1="1.143" y1="0.762" x2="1.27" y2="0.889" width="0.127" layer="21"/>
<wire x1="1.143" y1="0.762" x2="-1.143" y2="0.762" width="0.127" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="1.27" y2="-0.889" width="0.127" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="-1.143" y2="-0.762" width="0.127" layer="21"/>
<wire x1="1.524" y1="0.889" x2="1.27" y2="0.889" width="0.127" layer="21"/>
<wire x1="1.524" y1="-0.889" x2="1.27" y2="-0.889" width="0.127" layer="21"/>
<wire x1="1.778" y1="0.6388" x2="1.778" y2="-0.6332" width="0.127" layer="21"/>
<wire x1="-1.7787" y1="0.6274" x2="-1.7787" y2="-0.6446" width="0.127" layer="21"/>
<circle x="-2.54" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="-1.59" y="-0.477" size="1.016" layer="25">&gt;Name</text>
<text x="-2.544" y="-1.908" size="1.016" layer="27">&gt;Value</text>
<text x="-0.4797" y="0.8527" size="0.254" layer="100">PaJa</text>
<rectangle x1="-2.1022" y1="-0.306" x2="-1.8124" y2="0.3068" layer="21"/>
<rectangle x1="1.8124" y1="-0.3068" x2="2.1022" y2="0.306" layer="21"/>
</package>
<package name="R-10">
<description>&lt;B&gt;Odpor&lt;/B&gt; - vel. 0207 - 0,6W - vetsi roztec</description>
<wire x1="-2.572" y1="1.016" x2="-2.699" y2="1.143" width="0.127" layer="21"/>
<wire x1="-2.572" y1="-1.016" x2="-2.699" y2="-1.143" width="0.127" layer="21"/>
<wire x1="2.572" y1="1.016" x2="2.699" y2="1.143" width="0.127" layer="21"/>
<wire x1="2.572" y1="1.016" x2="-2.572" y2="1.016" width="0.127" layer="21"/>
<wire x1="2.572" y1="-1.016" x2="2.699" y2="-1.143" width="0.127" layer="21"/>
<wire x1="2.572" y1="-1.016" x2="-2.572" y2="-1.016" width="0.127" layer="21"/>
<wire x1="3.08" y1="1.139" x2="2.699" y2="1.139" width="0.127" layer="21"/>
<wire x1="3.08" y1="-1.147" x2="2.699" y2="-1.147" width="0.127" layer="21"/>
<wire x1="-3.334" y1="0.893" x2="-3.08" y2="1.147" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-3.334" y1="-0.885" x2="-3.08" y2="-1.139" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="-3.08" y1="-1.139" x2="-2.699" y2="-1.139" width="0.127" layer="21"/>
<wire x1="-3.08" y1="1.147" x2="-2.699" y2="1.147" width="0.127" layer="21"/>
<wire x1="-3.3321" y1="0.8823" x2="-3.3321" y2="-0.8667" width="0.127" layer="21"/>
<wire x1="3.08" y1="-1.147" x2="3.334" y2="-0.893" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="3.08" y1="1.139" x2="3.334" y2="0.885" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="3.3321" y1="-0.8823" x2="3.3321" y2="0.8667" width="0.127" layer="21"/>
<circle x="-5.08" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="-3.1152" y="-0.6276" size="1.27" layer="25">&gt;Name</text>
<text x="-0.3178" y="-0.6358" size="1.27" layer="27">&gt;Value</text>
<text x="2.3342" y="-0.9351" size="0.254" layer="100">PaJa</text>
<rectangle x1="-4.611" y1="-0.318" x2="-3.339" y2="0.318" layer="21"/>
<rectangle x1="3.339" y1="-0.318" x2="4.611" y2="0.318" layer="21"/>
</package>
<package name="R-12,7">
<description>&lt;B&gt;Odpor&lt;/B&gt; - roztec nozek 12,7mm</description>
<wire x1="3.7648" y1="1.2546" x2="3.8918" y2="1.3816" width="0.127" layer="21"/>
<wire x1="3.7648" y1="-1.2546" x2="3.8918" y2="-1.3816" width="0.127" layer="21"/>
<wire x1="4.2728" y1="1.3776" x2="3.8918" y2="1.3776" width="0.127" layer="21"/>
<wire x1="4.2728" y1="-1.3856" x2="3.8918" y2="-1.3856" width="0.127" layer="21"/>
<wire x1="4.2728" y1="-1.3856" x2="4.5268" y2="-1.1316" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="4.2728" y1="1.3776" x2="4.5268" y2="1.1236" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="4.5249" y1="-1.1209" x2="4.5249" y2="1.1053" width="0.127" layer="21"/>
<wire x1="-3.7649" y1="1.2547" x2="-3.8919" y2="1.3817" width="0.127" layer="21"/>
<wire x1="-3.7649" y1="-1.2546" x2="-3.8919" y2="-1.3816" width="0.127" layer="21"/>
<wire x1="3.7648" y1="1.2546" x2="-3.7649" y2="1.2547" width="0.127" layer="21"/>
<wire x1="3.7648" y1="-1.2546" x2="-3.7649" y2="-1.2546" width="0.127" layer="21"/>
<wire x1="-4.5269" y1="1.1316" x2="-4.2729" y2="1.3856" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-4.5269" y1="-1.1236" x2="-4.2729" y2="-1.3776" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="-4.2729" y1="-1.3776" x2="-3.8919" y2="-1.3776" width="0.127" layer="21"/>
<wire x1="-4.2729" y1="1.3856" x2="-3.8919" y2="1.3856" width="0.127" layer="21"/>
<wire x1="-4.525" y1="1.1209" x2="-4.525" y2="-1.1054" width="0.127" layer="21"/>
<circle x="-6.35" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="6.35" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" diameter="2.1844" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" diameter="2.1844" shape="octagon"/>
<text x="-0.4813" y="-0.7958" size="1.4224" layer="27">&gt;Value</text>
<text x="-4.2905" y="-0.7144" size="1.4224" layer="25">&gt;Name</text>
<text x="3.5712" y="-1.1046" size="0.254" layer="100">PaJa</text>
<rectangle x1="-5.8738" y1="-0.3175" x2="-4.5244" y2="0.3175" layer="21"/>
<rectangle x1="4.5244" y1="-0.3175" x2="5.8738" y2="0.3175" layer="21"/>
</package>
<package name="R-7,5">
<description>&lt;B&gt;Odpor&lt;/B&gt; - vel. 0207 - 0,6W</description>
<wire x1="-3.175" y1="0.893" x2="-2.921" y2="1.147" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-3.175" y1="-0.885" x2="-2.921" y2="-1.139" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="2.413" y1="-1.012" x2="2.54" y2="-1.139" width="0.127" layer="21"/>
<wire x1="2.413" y1="1.02" x2="2.54" y2="1.147" width="0.127" layer="21"/>
<wire x1="-2.413" y1="-1.012" x2="-2.54" y2="-1.139" width="0.127" layer="21"/>
<wire x1="-2.413" y1="-1.012" x2="2.413" y2="-1.012" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.02" x2="-2.54" y2="1.147" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.02" x2="2.413" y2="1.02" width="0.127" layer="21"/>
<wire x1="-2.921" y1="-1.139" x2="-2.54" y2="-1.139" width="0.127" layer="21"/>
<wire x1="-2.921" y1="1.147" x2="-2.54" y2="1.147" width="0.127" layer="21"/>
<wire x1="-3.1731" y1="0.8823" x2="-3.1731" y2="-0.8667" width="0.127" layer="21"/>
<wire x1="2.921" y1="-1.147" x2="3.175" y2="-0.893" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="2.921" y1="1.139" x2="3.175" y2="0.885" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="2.921" y1="1.139" x2="2.54" y2="1.139" width="0.127" layer="21"/>
<wire x1="2.921" y1="-1.147" x2="2.54" y2="-1.147" width="0.127" layer="21"/>
<wire x1="3.1731" y1="-0.8823" x2="3.1731" y2="0.8667" width="0.127" layer="21"/>
<circle x="-3.81" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="3.81" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="-0.3178" y="-0.477" size="1.016" layer="27">&gt;Value</text>
<text x="-2.7033" y="-0.477" size="1.016" layer="25">&gt;Name</text>
<text x="2.1354" y="-0.8658" size="0.254" layer="100">PaJa</text>
<rectangle x1="-3.4323" y1="-0.3053" x2="-3.1758" y2="0.3061" layer="21"/>
<rectangle x1="3.1759" y1="-0.3061" x2="3.4324" y2="0.3053" layer="21"/>
</package>
<package name="R-_2W">
<description>&lt;B&gt;Odpor&lt;/B&gt; - 2W - vel. 0414</description>
<wire x1="4.3998" y1="1.8896" x2="4.5268" y2="2.0166" width="0.127" layer="21"/>
<wire x1="4.3998" y1="-1.8896" x2="4.5268" y2="-2.0166" width="0.127" layer="21"/>
<wire x1="4.9078" y1="2.0126" x2="4.5268" y2="2.0126" width="0.127" layer="21"/>
<wire x1="4.9078" y1="-2.0206" x2="4.5268" y2="-2.0206" width="0.127" layer="21"/>
<wire x1="4.9078" y1="-2.0206" x2="5.1618" y2="-1.7666" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="4.9078" y1="2.0126" x2="5.1618" y2="1.7586" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="5.1599" y1="-1.7559" x2="5.1599" y2="1.7403" width="0.127" layer="21"/>
<wire x1="-4.3999" y1="1.8897" x2="-4.5269" y2="2.0167" width="0.127" layer="21"/>
<wire x1="-4.3999" y1="-1.8896" x2="-4.5269" y2="-2.0166" width="0.127" layer="21"/>
<wire x1="4.3998" y1="1.8896" x2="-4.3999" y2="1.8897" width="0.127" layer="21"/>
<wire x1="4.3998" y1="-1.8896" x2="-4.3999" y2="-1.8896" width="0.127" layer="21"/>
<wire x1="-5.1619" y1="1.7666" x2="-4.9079" y2="2.0206" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-5.1619" y1="-1.7586" x2="-4.9079" y2="-2.0126" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="-4.9079" y1="-2.0126" x2="-4.5269" y2="-2.0126" width="0.127" layer="21"/>
<wire x1="-4.9079" y1="2.0206" x2="-4.5269" y2="2.0206" width="0.127" layer="21"/>
<wire x1="-5.16" y1="1.7559" x2="-5.16" y2="-1.7404" width="0.127" layer="21"/>
<circle x="-6.35" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="6.35" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" diameter="2.54" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" diameter="2.54" shape="octagon"/>
<text x="-0.4813" y="-0.7958" size="1.6764" layer="27">&gt;Value</text>
<text x="-4.9255" y="-0.7144" size="1.6764" layer="25">&gt;Name</text>
<text x="4.1268" y="-1.7396" size="0.254" layer="100">PaJa</text>
<text x="-4.7625" y="-1.5875" size="0.6096" layer="21">2W</text>
<rectangle x1="-5.953" y1="-0.3175" x2="-5.1593" y2="0.3175" layer="21"/>
<rectangle x1="5.1594" y1="-0.3175" x2="5.9531" y2="0.3175" layer="21"/>
</package>
<package name="R-_10W">
<description>&lt;B&gt;Odpor&lt;/B&gt; - 10W - dratovy</description>
<wire x1="-24.13" y1="5.3975" x2="-24.13" y2="-5.3975" width="0.127" layer="21"/>
<wire x1="-24.13" y1="-5.3975" x2="24.13" y2="-5.3975" width="0.127" layer="21"/>
<wire x1="24.13" y1="-5.3975" x2="24.13" y2="5.3975" width="0.127" layer="21"/>
<wire x1="24.13" y1="5.3975" x2="-24.13" y2="5.3975" width="0.127" layer="21"/>
<circle x="-25.7175" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="25.7175" y="0" radius="0.5723" width="0.127" layer="102"/>
<pad name="1" x="-25.7175" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="25.7175" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-4.9375" y="-3.08" size="1.9304" layer="27">&gt;VALUE</text>
<text x="-4.9375" y="1.2225" size="1.9304" layer="25">&gt;NAME</text>
<text x="22.86" y="-5.08" size="0.254" layer="100">PaJa</text>
<text x="-23.1775" y="-3.81" size="1.27" layer="21">10W</text>
<rectangle x1="-25.2412" y1="-0.635" x2="-24.1299" y2="0.635" layer="21"/>
<rectangle x1="-25.5587" y1="0.4763" x2="-25.2412" y2="0.635" layer="21"/>
<rectangle x1="-25.5587" y1="-0.6349" x2="-25.2412" y2="-0.4762" layer="21"/>
<rectangle x1="24.13" y1="-0.635" x2="25.2413" y2="0.635" layer="21"/>
<rectangle x1="25.2413" y1="-0.6349" x2="25.5588" y2="-0.4762" layer="21"/>
<rectangle x1="25.2413" y1="0.4763" x2="25.5588" y2="0.635" layer="21"/>
</package>
<package name="R-_20W">
<description>&lt;B&gt;Odpor&lt;/B&gt; - 20W - dratovy</description>
<wire x1="-30.1625" y1="6.985" x2="-30.1625" y2="-6.985" width="0.127" layer="21"/>
<wire x1="-30.1625" y1="-6.985" x2="30.1625" y2="-6.985" width="0.127" layer="21"/>
<wire x1="30.1625" y1="-6.985" x2="30.1625" y2="6.985" width="0.127" layer="21"/>
<wire x1="30.1625" y1="6.985" x2="-30.1625" y2="6.985" width="0.127" layer="21"/>
<circle x="-31.75" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="31.75" y="0" radius="0.7099" width="0.127" layer="102"/>
<pad name="1" x="-31.75" y="0" drill="1.27" diameter="3.2" shape="octagon"/>
<pad name="2" x="31.75" y="0" drill="1.27" diameter="3.2" shape="octagon"/>
<text x="-4.9375" y="-4.6675" size="2.1844" layer="27">&gt;VALUE</text>
<text x="-4.9375" y="1.2225" size="2.1844" layer="25">&gt;NAME</text>
<text x="28.8925" y="-6.6675" size="0.254" layer="100">PaJa</text>
<text x="-29.21" y="-5.715" size="1.27" layer="21">20W</text>
<rectangle x1="30.1625" y1="-0.635" x2="31.115" y2="0.635" layer="21"/>
<rectangle x1="-31.1149" y1="-0.635" x2="-30.1624" y2="0.635" layer="21"/>
<rectangle x1="-31.2738" y1="0.4763" x2="-31.115" y2="0.635" layer="21"/>
<rectangle x1="-31.2738" y1="-0.6349" x2="-31.115" y2="-0.4762" layer="21"/>
<rectangle x1="31.115" y1="-0.6349" x2="31.2738" y2="-0.4762" layer="21"/>
<rectangle x1="31.115" y1="0.4763" x2="31.2738" y2="0.635" layer="21"/>
</package>
<package name="R-_5W">
<description>&lt;B&gt;Odpor&lt;/B&gt; - 5W - keramicky</description>
<wire x1="-11.1125" y1="5.08" x2="-11.1125" y2="-5.08" width="0.127" layer="21"/>
<wire x1="-11.1125" y1="-5.08" x2="11.1125" y2="-5.08" width="0.127" layer="21"/>
<wire x1="11.1125" y1="-5.08" x2="11.1125" y2="5.08" width="0.127" layer="21"/>
<wire x1="11.1125" y1="5.08" x2="-11.1125" y2="5.08" width="0.127" layer="21"/>
<circle x="-12.7" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="12.7" y="0" radius="0.5723" width="0.127" layer="102"/>
<pad name="1" x="-12.7" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="12.7" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-4.9375" y="-3.08" size="1.9304" layer="27">&gt;VALUE</text>
<text x="-4.9375" y="1.2225" size="1.9304" layer="25">&gt;NAME</text>
<text x="-10.16" y="-3.81" size="1.27" layer="21">5W</text>
<text x="9.8425" y="-4.7625" size="0.254" layer="100">PaJa</text>
<rectangle x1="-12.2237" y1="-0.635" x2="-11.1124" y2="0.635" layer="21"/>
<rectangle x1="-12.5412" y1="0.4763" x2="-12.2237" y2="0.635" layer="21"/>
<rectangle x1="-12.5412" y1="-0.6349" x2="-12.2237" y2="-0.4762" layer="21"/>
<rectangle x1="11.1125" y1="-0.635" x2="12.2238" y2="0.635" layer="21"/>
<rectangle x1="12.2238" y1="-0.6349" x2="12.5413" y2="-0.4762" layer="21"/>
<rectangle x1="12.2238" y1="0.4763" x2="12.5413" y2="0.635" layer="21"/>
</package>
<package name="R-STOJ">
<description>&lt;B&gt;Odpor&lt;/B&gt; - vel. 0207 - 0,6W - nastojato</description>
<circle x="-1.272" y="0" radius="1.2818" width="0.127" layer="21"/>
<circle x="-1.27" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="1.27" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<text x="-2.389" y="1.433" size="1.016" layer="25">&gt;Name</text>
<text x="-2.544" y="-2.385" size="1.016" layer="27">&gt;Value</text>
<text x="0.636" y="-1.272" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-0.795" y1="-0.318" x2="0.795" y2="0.318" layer="21"/>
</package>
<package name="S1G2_JUM">
<wire x1="0" y1="1.016" x2="0.254" y2="1.27" width="0.127" layer="21"/>
<wire x1="0" y1="1.016" x2="-0.254" y2="1.27" width="0.127" layer="21"/>
<wire x1="0" y1="-1.016" x2="0.254" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0" y1="-1.016" x2="-0.254" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.27" x2="-2.286" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-2.54" y1="-1.016" x2="-2.286" y2="-1.27" width="0.127" layer="21"/>
<wire x1="2.286" y1="-1.27" x2="2.54" y2="-1.016" width="0.127" layer="21"/>
<wire x1="2.286" y1="-1.27" x2="0.254" y2="-1.27" width="0.127" layer="21"/>
<wire x1="2.54" y1="-1.016" x2="2.54" y2="1.016" width="0.127" layer="21"/>
<wire x1="2.286" y1="1.27" x2="2.54" y2="1.016" width="0.127" layer="21"/>
<wire x1="2.286" y1="1.27" x2="0.254" y2="1.27" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.27" x2="-2.286" y2="1.27" width="0.127" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.286" y2="1.27" width="0.127" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.54" y2="-1.016" width="0.127" layer="21"/>
<wire x1="0" y1="0.954" x2="0" y2="-0.954" width="0.127" layer="21"/>
<circle x="-1.27" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="1.27" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="1" x="-1.27" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<text x="-2.54" y="1.492" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.762" size="1.27" layer="27">&gt;VALUE</text>
<text x="-0.159" y="-0.954" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="0.9652" y1="-0.3048" x2="1.5748" y2="0.3048" layer="51"/>
<rectangle x1="-1.5748" y1="-0.3048" x2="-0.9652" y2="0.3048" layer="51"/>
</package>
<package name="S1G3_JUM">
<wire x1="-1.27" y1="1.016" x2="-1.016" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="1.016" x2="-1.524" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-1.016" x2="-1.016" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-1.016" x2="-1.524" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.524" y1="-1.27" x2="-3.556" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-3.81" y1="-1.016" x2="-3.556" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.016" y1="-1.27" x2="1.27" y2="-1.016" width="0.127" layer="21"/>
<wire x1="1.016" y1="-1.27" x2="-1.016" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.016" y1="1.27" x2="1.27" y2="1.016" width="0.127" layer="21"/>
<wire x1="1.016" y1="1.27" x2="-1.016" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.524" y1="1.27" x2="-3.556" y2="1.27" width="0.127" layer="21"/>
<wire x1="-3.81" y1="1.016" x2="-3.556" y2="1.27" width="0.127" layer="21"/>
<wire x1="-3.81" y1="1.016" x2="-3.81" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-1.27" y1="0.954" x2="-1.27" y2="-0.954" width="0.127" layer="21"/>
<wire x1="3.818" y1="1.016" x2="3.564" y2="1.27" width="0.127" layer="21"/>
<wire x1="3.818" y1="-1.016" x2="3.564" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.564" y1="-1.27" x2="1.532" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.278" y1="-1.016" x2="1.532" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.564" y1="1.27" x2="1.532" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.278" y1="1.016" x2="1.532" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.278" y1="1.016" x2="1.278" y2="-1.016" width="0.127" layer="21"/>
<wire x1="3.818" y1="0.954" x2="3.818" y2="-0.954" width="0.127" layer="21"/>
<circle x="-2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="0" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="1" x="-2.54" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<text x="-2.856" y="1.492" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.333" y="-2.762" size="1.27" layer="27">&gt;VALUE</text>
<text x="1.115" y="-0.954" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-0.3048" y1="-0.3048" x2="0.3048" y2="0.3048" layer="51"/>
<rectangle x1="-2.8448" y1="-0.3048" x2="-2.2352" y2="0.3048" layer="51"/>
<rectangle x1="2.2432" y1="-0.3048" x2="2.8528" y2="0.3048" layer="51"/>
</package>
<package name="PAD_1">
<circle x="0" y="0" radius="0.5732" width="0.127" layer="102"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="2.1844" shape="octagon"/>
<text x="-2.226" y="1.272" size="1.27" layer="25">&gt;Name</text>
<text x="-2.067" y="-2.544" size="1.27" layer="27">&gt;Value</text>
<text x="-1.272" y="-0.477" size="0.254" layer="100" rot="R90">PaJa</text>
</package>
<package name="S1G5_JUM">
<wire x1="-3.814" y1="1.016" x2="-3.56" y2="1.27" width="0.127" layer="21"/>
<wire x1="-3.814" y1="1.016" x2="-4.068" y2="1.27" width="0.127" layer="21"/>
<wire x1="-3.814" y1="-1.016" x2="-3.56" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-3.814" y1="-1.016" x2="-4.068" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-4.068" y1="-1.27" x2="-6.1" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-6.354" y1="-1.016" x2="-6.1" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.528" y1="-1.27" x2="-1.274" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-1.528" y1="-1.27" x2="-3.56" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.528" y1="1.27" x2="-1.274" y2="1.016" width="0.127" layer="21"/>
<wire x1="-1.528" y1="1.27" x2="-3.56" y2="1.27" width="0.127" layer="21"/>
<wire x1="-4.068" y1="1.27" x2="-6.1" y2="1.27" width="0.127" layer="21"/>
<wire x1="-6.354" y1="1.016" x2="-6.1" y2="1.27" width="0.127" layer="21"/>
<wire x1="-6.354" y1="1.016" x2="-6.354" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-3.814" y1="0.954" x2="-3.814" y2="-0.954" width="0.127" layer="21"/>
<wire x1="1.274" y1="1.016" x2="1.528" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.274" y1="1.016" x2="1.02" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.274" y1="-1.016" x2="1.528" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.274" y1="-1.016" x2="1.02" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.02" y1="-1.27" x2="-1.012" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.266" y1="-1.016" x2="-1.012" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.56" y1="-1.27" x2="3.814" y2="-1.016" width="0.127" layer="21"/>
<wire x1="3.56" y1="-1.27" x2="1.528" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.56" y1="1.27" x2="3.814" y2="1.016" width="0.127" layer="21"/>
<wire x1="3.56" y1="1.27" x2="1.528" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.02" y1="1.27" x2="-1.012" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.266" y1="1.016" x2="-1.012" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.266" y1="1.016" x2="-1.266" y2="-1.016" width="0.127" layer="21"/>
<wire x1="1.274" y1="0.954" x2="1.274" y2="-0.954" width="0.127" layer="21"/>
<wire x1="6.346" y1="-1.016" x2="6.092" y2="-1.27" width="0.127" layer="21"/>
<wire x1="6.092" y1="-1.27" x2="4.06" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.806" y1="-1.016" x2="4.06" y2="-1.27" width="0.127" layer="21"/>
<wire x1="6.092" y1="1.27" x2="4.06" y2="1.27" width="0.127" layer="21"/>
<wire x1="3.806" y1="1.016" x2="4.06" y2="1.27" width="0.127" layer="21"/>
<wire x1="3.806" y1="1.016" x2="3.806" y2="-1.016" width="0.127" layer="21"/>
<wire x1="6.354" y1="1.016" x2="6.1" y2="1.27" width="0.127" layer="21"/>
<wire x1="6.354" y1="0.954" x2="6.354" y2="-0.954" width="0.127" layer="21"/>
<circle x="-5.08" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="-2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="0" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="1" x="-5.08" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="2" x="-2.54" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="3" x="0" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="4" x="2.54" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="5" x="5.08" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<text x="-2.858" y="1.492" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.0175" y="-2.762" size="1.27" layer="27">&gt;VALUE</text>
<text x="1.111" y="-0.795" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-2.8488" y1="-0.3048" x2="-2.2392" y2="0.3048" layer="51"/>
<rectangle x1="-5.3888" y1="-0.3048" x2="-4.7792" y2="0.3048" layer="51"/>
<rectangle x1="2.2392" y1="-0.3048" x2="2.8488" y2="0.3048" layer="51"/>
<rectangle x1="-0.3008" y1="-0.3048" x2="0.3088" y2="0.3048" layer="51"/>
<rectangle x1="4.7712" y1="-0.3048" x2="5.3808" y2="0.3048" layer="51"/>
</package>
<package name="S1G7_JUM">
<wire x1="-6.354" y1="1.016" x2="-6.1" y2="1.27" width="0.127" layer="21"/>
<wire x1="-6.354" y1="1.016" x2="-6.608" y2="1.27" width="0.127" layer="21"/>
<wire x1="-6.354" y1="-1.016" x2="-6.1" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-6.354" y1="-1.016" x2="-6.608" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-6.608" y1="-1.27" x2="-8.64" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-8.894" y1="-1.016" x2="-8.64" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-4.068" y1="-1.27" x2="-3.814" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-4.068" y1="-1.27" x2="-6.1" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-4.068" y1="1.27" x2="-3.814" y2="1.016" width="0.127" layer="21"/>
<wire x1="-4.068" y1="1.27" x2="-6.1" y2="1.27" width="0.127" layer="21"/>
<wire x1="-6.608" y1="1.27" x2="-8.64" y2="1.27" width="0.127" layer="21"/>
<wire x1="-8.894" y1="1.016" x2="-8.64" y2="1.27" width="0.127" layer="21"/>
<wire x1="-8.894" y1="1.016" x2="-8.894" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-6.354" y1="0.954" x2="-6.354" y2="-0.954" width="0.127" layer="21"/>
<wire x1="-1.266" y1="1.016" x2="-1.012" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.266" y1="1.016" x2="-1.52" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.266" y1="-1.016" x2="-1.012" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.266" y1="-1.016" x2="-1.52" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.52" y1="-1.27" x2="-3.552" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-3.806" y1="-1.016" x2="-3.552" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.02" y1="-1.27" x2="1.274" y2="-1.016" width="0.127" layer="21"/>
<wire x1="1.02" y1="-1.27" x2="-1.012" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.02" y1="1.27" x2="1.274" y2="1.016" width="0.127" layer="21"/>
<wire x1="1.02" y1="1.27" x2="-1.012" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.52" y1="1.27" x2="-3.552" y2="1.27" width="0.127" layer="21"/>
<wire x1="-3.806" y1="1.016" x2="-3.552" y2="1.27" width="0.127" layer="21"/>
<wire x1="-3.806" y1="1.016" x2="-3.806" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-1.266" y1="0.954" x2="-1.266" y2="-0.954" width="0.127" layer="21"/>
<wire x1="3.806" y1="-1.016" x2="3.552" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.552" y1="-1.27" x2="1.52" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.266" y1="-1.016" x2="1.52" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.552" y1="1.27" x2="1.52" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.266" y1="1.016" x2="1.52" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.266" y1="1.016" x2="1.266" y2="-1.016" width="0.127" layer="21"/>
<wire x1="3.814" y1="1.016" x2="3.56" y2="1.27" width="0.127" layer="21"/>
<wire x1="6.346" y1="1.016" x2="6.6" y2="1.27" width="0.127" layer="21"/>
<wire x1="6.346" y1="1.016" x2="6.092" y2="1.27" width="0.127" layer="21"/>
<wire x1="6.346" y1="-1.016" x2="6.6" y2="-1.27" width="0.127" layer="21"/>
<wire x1="6.346" y1="-1.016" x2="6.092" y2="-1.27" width="0.127" layer="21"/>
<wire x1="6.092" y1="-1.27" x2="4.06" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.806" y1="-1.016" x2="4.06" y2="-1.27" width="0.127" layer="21"/>
<wire x1="8.632" y1="-1.27" x2="8.886" y2="-1.016" width="0.127" layer="21"/>
<wire x1="8.632" y1="-1.27" x2="6.6" y2="-1.27" width="0.127" layer="21"/>
<wire x1="8.632" y1="1.27" x2="8.886" y2="1.016" width="0.127" layer="21"/>
<wire x1="8.632" y1="1.27" x2="6.6" y2="1.27" width="0.127" layer="21"/>
<wire x1="6.092" y1="1.27" x2="4.06" y2="1.27" width="0.127" layer="21"/>
<wire x1="3.806" y1="1.016" x2="4.06" y2="1.27" width="0.127" layer="21"/>
<wire x1="3.806" y1="1.016" x2="3.806" y2="-1.016" width="0.127" layer="21"/>
<wire x1="6.346" y1="0.954" x2="6.346" y2="-0.954" width="0.127" layer="21"/>
<wire x1="8.894" y1="1.016" x2="8.894" y2="-1.016" width="0.127" layer="21"/>
<circle x="-7.62" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="-5.08" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="-2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="0" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="7.62" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="1" x="-7.62" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="2" x="-5.08" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="3" x="-2.54" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="4" x="0" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="5" x="2.54" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="6" x="5.08" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="7" x="7.62" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<text x="-2.5405" y="1.492" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.0175" y="-2.762" size="1.27" layer="27">&gt;VALUE</text>
<text x="1.111" y="-0.795" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-5.3888" y1="-0.3048" x2="-4.7792" y2="0.3048" layer="51"/>
<rectangle x1="-7.9288" y1="-0.3048" x2="-7.3192" y2="0.3048" layer="51"/>
<rectangle x1="-0.3008" y1="-0.3048" x2="0.3088" y2="0.3048" layer="51"/>
<rectangle x1="-2.8408" y1="-0.3048" x2="-2.2312" y2="0.3048" layer="51"/>
<rectangle x1="2.2312" y1="-0.3048" x2="2.8408" y2="0.3048" layer="51"/>
<rectangle x1="7.3112" y1="-0.3048" x2="7.9208" y2="0.3048" layer="51"/>
<rectangle x1="4.7712" y1="-0.3048" x2="5.3808" y2="0.3048" layer="51"/>
</package>
<package name="S1G4_JUM">
<wire x1="-2.544" y1="1.016" x2="-2.29" y2="1.27" width="0.127" layer="21"/>
<wire x1="-2.544" y1="1.016" x2="-2.798" y2="1.27" width="0.127" layer="21"/>
<wire x1="-2.544" y1="-1.016" x2="-2.29" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-2.544" y1="-1.016" x2="-2.798" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-2.798" y1="-1.27" x2="-4.83" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-5.084" y1="-1.016" x2="-4.83" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-0.258" y1="-1.27" x2="-0.004" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-0.258" y1="-1.27" x2="-2.29" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-0.258" y1="1.27" x2="-0.004" y2="1.016" width="0.127" layer="21"/>
<wire x1="-0.258" y1="1.27" x2="-2.29" y2="1.27" width="0.127" layer="21"/>
<wire x1="-2.798" y1="1.27" x2="-4.83" y2="1.27" width="0.127" layer="21"/>
<wire x1="-5.084" y1="1.016" x2="-4.83" y2="1.27" width="0.127" layer="21"/>
<wire x1="-5.084" y1="1.016" x2="-5.084" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-2.544" y1="0.954" x2="-2.544" y2="-0.954" width="0.127" layer="21"/>
<wire x1="2.544" y1="1.016" x2="2.798" y2="1.27" width="0.127" layer="21"/>
<wire x1="2.544" y1="1.016" x2="2.29" y2="1.27" width="0.127" layer="21"/>
<wire x1="2.544" y1="-1.016" x2="2.798" y2="-1.27" width="0.127" layer="21"/>
<wire x1="2.544" y1="-1.016" x2="2.29" y2="-1.27" width="0.127" layer="21"/>
<wire x1="2.29" y1="-1.27" x2="0.258" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0.004" y1="-1.016" x2="0.258" y2="-1.27" width="0.127" layer="21"/>
<wire x1="4.83" y1="-1.27" x2="5.084" y2="-1.016" width="0.127" layer="21"/>
<wire x1="4.83" y1="-1.27" x2="2.798" y2="-1.27" width="0.127" layer="21"/>
<wire x1="5.084" y1="-1.016" x2="5.084" y2="1.016" width="0.127" layer="21"/>
<wire x1="4.83" y1="1.27" x2="5.084" y2="1.016" width="0.127" layer="21"/>
<wire x1="4.83" y1="1.27" x2="2.798" y2="1.27" width="0.127" layer="21"/>
<wire x1="2.29" y1="1.27" x2="0.258" y2="1.27" width="0.127" layer="21"/>
<wire x1="0.004" y1="1.016" x2="0.258" y2="1.27" width="0.127" layer="21"/>
<wire x1="0.004" y1="1.016" x2="0.004" y2="-1.016" width="0.127" layer="21"/>
<wire x1="2.544" y1="0.954" x2="2.544" y2="-0.954" width="0.127" layer="21"/>
<circle x="-3.81" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="-1.27" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="1.27" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="3.81" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="1" x="-3.81" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="2" x="-1.27" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="3" x="1.27" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="4" x="3.81" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<text x="-2.858" y="1.492" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.335" y="-2.762" size="1.27" layer="27">&gt;VALUE</text>
<text x="-0.159" y="-0.795" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-1.5788" y1="-0.3048" x2="-0.9692" y2="0.3048" layer="51"/>
<rectangle x1="-4.1188" y1="-0.3048" x2="-3.5092" y2="0.3048" layer="51"/>
<rectangle x1="3.5092" y1="-0.3048" x2="4.1188" y2="0.3048" layer="51"/>
<rectangle x1="0.9692" y1="-0.3048" x2="1.5788" y2="0.3048" layer="51"/>
</package>
<package name="MLW10A">
<wire x1="-6.35" y1="10.16" x2="-3.81" y2="10.16" width="0.254" layer="21"/>
<wire x1="-3.81" y1="10.16" x2="-5.08" y2="7.62" width="0.254" layer="21"/>
<wire x1="-5.08" y1="7.62" x2="-6.35" y2="10.16" width="0.254" layer="21"/>
<wire x1="2.794" y1="9.906" x2="2.794" y2="10.922" width="0.127" layer="21"/>
<wire x1="2.794" y1="9.906" x2="4.826" y2="9.906" width="0.127" layer="21"/>
<wire x1="4.826" y1="10.922" x2="4.826" y2="9.906" width="0.127" layer="21"/>
<wire x1="2.159" y1="3.683" x2="3.048" y2="3.683" width="0.127" layer="21"/>
<wire x1="3.048" y1="5.969" x2="4.572" y2="5.969" width="0.127" layer="21" curve="-180"/>
<wire x1="4.572" y1="5.969" x2="4.572" y2="4.445" width="0.127" layer="21"/>
<wire x1="4.572" y1="3.683" x2="5.969" y2="3.683" width="0.127" layer="21"/>
<wire x1="-2.159" y1="10.922" x2="2.159" y2="10.922" width="0.127" layer="21"/>
<wire x1="2.159" y1="3.429" x2="-2.159" y2="3.429" width="0.127" layer="21"/>
<wire x1="-2.159" y1="10.922" x2="-2.159" y2="3.429" width="0.127" layer="21"/>
<wire x1="2.159" y1="10.922" x2="2.159" y2="4.445" width="0.127" layer="21"/>
<wire x1="2.159" y1="4.445" x2="2.159" y2="3.683" width="0.127" layer="21"/>
<wire x1="2.159" y1="3.683" x2="2.159" y2="3.429" width="0.127" layer="21"/>
<wire x1="2.159" y1="3.429" x2="2.159" y2="2.159" width="0.127" layer="21"/>
<wire x1="3.048" y1="5.969" x2="3.048" y2="4.445" width="0.127" layer="21"/>
<wire x1="3.048" y1="4.445" x2="3.048" y2="3.683" width="0.127" layer="21"/>
<wire x1="3.048" y1="4.445" x2="4.572" y2="4.445" width="0.127" layer="21"/>
<wire x1="4.572" y1="4.445" x2="4.572" y2="3.683" width="0.127" layer="21"/>
<wire x1="5.969" y1="3.683" x2="5.969" y2="10.922" width="0.127" layer="21"/>
<wire x1="5.969" y1="3.683" x2="5.969" y2="2.032" width="0.127" layer="21"/>
<wire x1="-5.715" y1="2.032" x2="-4.445" y2="2.032" width="0.127" layer="51"/>
<wire x1="-4.445" y1="2.032" x2="-3.175" y2="2.032" width="0.127" layer="21"/>
<wire x1="-3.175" y1="2.032" x2="-1.905" y2="2.032" width="0.127" layer="51"/>
<wire x1="-1.905" y1="2.032" x2="-0.635" y2="2.032" width="0.127" layer="21"/>
<wire x1="-0.635" y1="2.032" x2="0.635" y2="2.032" width="0.127" layer="51"/>
<wire x1="0.635" y1="2.032" x2="1.905" y2="2.032" width="0.127" layer="21"/>
<wire x1="1.905" y1="2.032" x2="2.159" y2="2.032" width="0.127" layer="51"/>
<wire x1="3.175" y1="2.032" x2="4.445" y2="2.032" width="0.127" layer="21"/>
<wire x1="5.715" y1="2.032" x2="4.445" y2="2.032" width="0.127" layer="51"/>
<wire x1="0" y1="10.033" x2="0" y2="10.287" width="0.508" layer="21"/>
<wire x1="9.525" y1="4.445" x2="9.525" y2="8.255" width="0.127" layer="21"/>
<wire x1="6.731" y1="8.255" x2="9.525" y2="8.255" width="0.127" layer="21"/>
<wire x1="6.731" y1="8.255" x2="6.731" y2="4.445" width="0.127" layer="21"/>
<wire x1="2.159" y1="10.922" x2="10.16" y2="10.922" width="0.127" layer="21"/>
<wire x1="10.16" y1="10.922" x2="10.16" y2="2.032" width="0.127" layer="21"/>
<wire x1="10.16" y1="2.032" x2="9.017" y2="2.032" width="0.127" layer="21"/>
<wire x1="-2.159" y1="10.922" x2="-10.16" y2="10.922" width="0.127" layer="21"/>
<wire x1="-10.16" y1="2.032" x2="-10.16" y2="10.922" width="0.127" layer="21"/>
<wire x1="-10.16" y1="2.032" x2="-9.017" y2="2.032" width="0.127" layer="21"/>
<wire x1="5.969" y1="2.032" x2="5.715" y2="2.032" width="0.127" layer="21"/>
<wire x1="2.159" y1="2.159" x2="2.159" y2="2.032" width="0.127" layer="51"/>
<wire x1="2.159" y1="2.032" x2="3.175" y2="2.032" width="0.127" layer="51"/>
<wire x1="8.001" y1="2.032" x2="8.001" y2="1.397" width="0.127" layer="21"/>
<wire x1="9.017" y1="1.397" x2="8.001" y2="1.397" width="0.127" layer="21"/>
<wire x1="9.017" y1="1.397" x2="9.017" y2="2.032" width="0.127" layer="21"/>
<wire x1="8.001" y1="2.032" x2="5.969" y2="2.032" width="0.127" layer="21"/>
<wire x1="9.017" y1="2.032" x2="8.001" y2="2.032" width="0.127" layer="21"/>
<wire x1="-9.017" y1="2.032" x2="-9.017" y2="1.397" width="0.127" layer="21"/>
<wire x1="-9.017" y1="2.032" x2="-8.001" y2="2.032" width="0.127" layer="21"/>
<wire x1="-8.001" y1="1.397" x2="-9.017" y2="1.397" width="0.127" layer="21"/>
<wire x1="-8.001" y1="1.397" x2="-8.001" y2="2.032" width="0.127" layer="21"/>
<wire x1="-8.001" y1="2.032" x2="-5.715" y2="2.032" width="0.127" layer="21"/>
<wire x1="2.159" y1="4.445" x2="-2.159" y2="4.445" width="0.127" layer="21"/>
<wire x1="6.731" y1="4.445" x2="9.525" y2="4.445" width="0.127" layer="21"/>
<circle x="-5.08" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="-2.54" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="0" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="2.54" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="5.08" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="5.08" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="2.54" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="0" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="-2.54" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="-5.08" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<pad name="1" x="-5.08" y="-1.27" drill="0.9144" diameter="1.778" shape="square"/>
<pad name="2" x="-5.08" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="3" x="-2.54" y="-1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="4" x="-2.54" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="5" x="0" y="-1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="6" x="0" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="7" x="2.54" y="-1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="8" x="2.54" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="9" x="5.08" y="-1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="10" x="5.08" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<text x="-10.1854" y="11.43" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.0254" y="11.43" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="8.8265" y="5.334" size="1.27" layer="101" ratio="10" rot="R90">10</text>
<text x="-7.3025" y="-1.905" size="1.27" layer="101" ratio="10">1</text>
<text x="-7.3025" y="0.3175" size="1.27" layer="101" ratio="10">2</text>
<text x="9.8425" y="2.54" size="0.254" layer="100" rot="R180">PaJa</text>
<rectangle x1="-0.254" y1="4.445" x2="0.254" y2="10.287" layer="21"/>
<rectangle x1="-6.223" y1="9.652" x2="-3.937" y2="10.16" layer="21"/>
<rectangle x1="-5.969" y1="9.144" x2="-4.191" y2="9.652" layer="21"/>
<rectangle x1="-5.715" y1="8.636" x2="-4.445" y2="9.144" layer="21"/>
<rectangle x1="-5.461" y1="8.128" x2="-4.699" y2="8.636" layer="21"/>
<rectangle x1="-5.207" y1="7.874" x2="-4.953" y2="8.128" layer="21"/>
<rectangle x1="-5.334" y1="-0.381" x2="-4.826" y2="0.381" layer="21"/>
<rectangle x1="-5.334" y1="0.381" x2="-4.826" y2="2.032" layer="51"/>
<rectangle x1="-5.334" y1="-1.524" x2="-4.826" y2="-0.381" layer="51"/>
<rectangle x1="-2.794" y1="0.381" x2="-2.286" y2="2.032" layer="51"/>
<rectangle x1="-2.794" y1="-0.381" x2="-2.286" y2="0.381" layer="21"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-0.381" layer="51"/>
<rectangle x1="-0.254" y1="0.381" x2="0.254" y2="2.032" layer="51"/>
<rectangle x1="-0.254" y1="-0.381" x2="0.254" y2="0.381" layer="21"/>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-0.381" layer="51"/>
<rectangle x1="2.286" y1="0.381" x2="2.794" y2="2.032" layer="51"/>
<rectangle x1="2.286" y1="-0.381" x2="2.794" y2="0.381" layer="21"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-0.381" layer="51"/>
<rectangle x1="4.826" y1="0.381" x2="5.334" y2="2.032" layer="51"/>
<rectangle x1="4.826" y1="-0.381" x2="5.334" y2="0.381" layer="21"/>
<rectangle x1="4.826" y1="-1.524" x2="5.334" y2="-0.381" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-5.334" y1="1.016" x2="-4.826" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="4.826" y1="1.016" x2="5.334" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="-5.334" y1="-1.524" x2="-4.826" y2="-1.016" layer="51"/>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="4.826" y1="-1.524" x2="5.334" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
</package>
<package name="MLW10G">
<wire x1="-8.89" y1="3.175" x2="8.89" y2="3.175" width="0.127" layer="21"/>
<wire x1="8.89" y1="-3.175" x2="8.89" y2="3.175" width="0.127" layer="21"/>
<wire x1="-8.89" y1="3.175" x2="-8.89" y2="-3.175" width="0.127" layer="21"/>
<wire x1="-10.16" y1="4.445" x2="-8.5725" y2="4.445" width="0.127" layer="21"/>
<wire x1="10.16" y1="-4.445" x2="10.16" y2="4.445" width="0.127" layer="21"/>
<wire x1="-10.16" y1="4.445" x2="-10.16" y2="-4.445" width="0.127" layer="21"/>
<wire x1="2.032" y1="-2.413" x2="-2.032" y2="-2.413" width="0.127" layer="21"/>
<wire x1="-2.032" y1="-3.175" x2="-2.032" y2="-2.413" width="0.127" layer="21"/>
<wire x1="-2.032" y1="-3.175" x2="-8.89" y2="-3.175" width="0.127" layer="21"/>
<wire x1="-2.032" y1="-3.175" x2="-2.032" y2="-3.429" width="0.127" layer="21"/>
<wire x1="2.032" y1="-2.413" x2="2.032" y2="-3.175" width="0.127" layer="21"/>
<wire x1="8.5725" y1="4.445" x2="10.16" y2="4.445" width="0.127" layer="21"/>
<wire x1="0.3175" y1="4.445" x2="7.9375" y2="4.445" width="0.127" layer="21"/>
<wire x1="-7.9375" y1="4.445" x2="-0.3175" y2="4.445" width="0.127" layer="21"/>
<wire x1="10.16" y1="-4.445" x2="2.032" y2="-4.445" width="0.127" layer="21"/>
<wire x1="2.032" y1="-4.445" x2="-2.032" y2="-4.445" width="0.127" layer="21"/>
<wire x1="8.89" y1="-3.175" x2="2.032" y2="-3.175" width="0.127" layer="21"/>
<wire x1="2.032" y1="-3.175" x2="2.032" y2="-3.429" width="0.127" layer="21"/>
<wire x1="2.032" y1="-3.429" x2="2.032" y2="-4.445" width="0.127" layer="21"/>
<wire x1="2.032" y1="-3.429" x2="9.144" y2="-3.429" width="0.0508" layer="21"/>
<wire x1="9.144" y1="-3.429" x2="9.144" y2="3.429" width="0.0508" layer="21"/>
<wire x1="9.144" y1="3.429" x2="-9.144" y2="3.429" width="0.0508" layer="21"/>
<wire x1="-9.144" y1="3.429" x2="-9.144" y2="-3.429" width="0.0508" layer="21"/>
<wire x1="-9.144" y1="-3.429" x2="-2.032" y2="-3.429" width="0.0508" layer="21"/>
<wire x1="-2.032" y1="-3.429" x2="-2.032" y2="-4.445" width="0.127" layer="21"/>
<wire x1="-2.032" y1="-4.445" x2="-4.445" y2="-4.445" width="0.127" layer="21"/>
<wire x1="-4.445" y1="-4.318" x2="-4.445" y2="-4.445" width="0.127" layer="21"/>
<wire x1="-4.445" y1="-4.318" x2="-5.715" y2="-4.318" width="0.127" layer="21"/>
<wire x1="-5.715" y1="-4.445" x2="-5.715" y2="-4.318" width="0.127" layer="21"/>
<wire x1="-5.715" y1="-4.445" x2="-10.16" y2="-4.445" width="0.127" layer="21"/>
<wire x1="-8.5725" y1="4.445" x2="-7.9375" y2="4.445" width="0.127" layer="21" curve="-180"/>
<wire x1="-0.3175" y1="4.445" x2="0.3175" y2="4.445" width="0.127" layer="21" curve="-180"/>
<wire x1="7.9375" y1="4.445" x2="8.5725" y2="4.445" width="0.127" layer="21" curve="-180"/>
<circle x="-5.08" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="-2.54" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="0" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="2.54" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="5.08" y="1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="5.08" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="2.54" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="0" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="-2.54" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<circle x="-5.08" y="-1.27" radius="0.449" width="0.127" layer="102"/>
<pad name="1" x="-5.08" y="-1.27" drill="0.9144" diameter="1.778" shape="square"/>
<pad name="2" x="-5.08" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="3" x="-2.54" y="-1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="4" x="-2.54" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="5" x="0" y="-1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="6" x="0" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="7" x="2.54" y="-1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="8" x="2.54" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="9" x="5.08" y="-1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<pad name="10" x="5.08" y="1.27" drill="0.9144" diameter="1.778" shape="octagon"/>
<text x="-1.016" y="-4.064" size="1.27" layer="101" ratio="10">10</text>
<text x="-8.255" y="5.08" size="1.778" layer="25">&gt;NAME</text>
<text x="0.3175" y="5.08" size="1.778" layer="27">&gt;VALUE</text>
<text x="-7.62" y="-1.905" size="1.27" layer="101" ratio="10">1</text>
<text x="-7.62" y="0.635" size="1.27" layer="101" ratio="10">2</text>
<text x="8.89" y="-4.1275" size="0.254" layer="100">PaJa</text>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-5.334" y1="1.016" x2="-4.826" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="4.826" y1="1.016" x2="5.334" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="-5.334" y1="-1.524" x2="-4.826" y2="-1.016" layer="51"/>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="4.826" y1="-1.524" x2="5.334" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="C-EL">
<wire x1="-3.8173" y1="0.9547" x2="-2.5453" y2="0.9547" width="0.155" layer="94"/>
<wire x1="-3.1812" y1="1.5908" x2="-3.1812" y2="0.3188" width="0.155" layer="94"/>
<wire x1="-2.0638" y1="1.7463" x2="-1.4288" y2="1.7463" width="0.254" layer="94"/>
<wire x1="-1.4288" y1="1.7463" x2="-1.4288" y2="-1.5875" width="0.254" layer="94"/>
<wire x1="-1.4288" y1="-1.5875" x2="-2.0638" y2="-1.5875" width="0.254" layer="94"/>
<wire x1="-2.0638" y1="-1.5875" x2="-2.0638" y2="0" width="0.254" layer="94"/>
<wire x1="-2.0638" y1="0" x2="-2.0638" y2="1.7463" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0" x2="-2.0638" y2="0" width="0.152" layer="94"/>
<wire x1="-0.4763" y1="0" x2="0" y2="0" width="0.152" layer="94"/>
<text x="-1.589" y="-0.477" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="0.3175" y="0.635" size="1.6764" layer="95">&gt;Name</text>
<text x="0.3175" y="-0.635" size="1.6764" layer="96" rot="MR180">&gt;Value</text>
<rectangle x1="-0.9525" y1="-1.7463" x2="-0.3175" y2="1.905" layer="94"/>
<pin name="C_EL+" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
<pin name="C_EL-" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="TLUMIVKA">
<wire x1="-5.08" y1="0" x2="-3.81" y2="1.27" width="0.254" layer="94" curve="-90" cap="flat"/>
<wire x1="-3.81" y1="1.27" x2="-2.54" y2="0" width="0.254" layer="94" curve="-90" cap="flat"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="1.27" width="0.254" layer="94" curve="-90" cap="flat"/>
<wire x1="-1.27" y1="1.27" x2="0" y2="0" width="0.254" layer="94" curve="-90" cap="flat"/>
<wire x1="0" y1="0" x2="1.27" y2="1.27" width="0.254" layer="94" curve="-90" cap="flat"/>
<wire x1="1.27" y1="1.27" x2="2.54" y2="0" width="0.254" layer="94" curve="-90" cap="flat"/>
<wire x1="2.54" y1="0" x2="3.81" y2="1.27" width="0.254" layer="94" curve="-90" cap="flat"/>
<wire x1="3.81" y1="1.27" x2="5.08" y2="0" width="0.254" layer="94" curve="-90" cap="flat"/>
<wire x1="-5.08" y1="2.54" x2="5.08" y2="2.54" width="0.6096" layer="94"/>
<text x="-4.445" y="-2.2225" size="1.778" layer="96">&gt;Value</text>
<text x="-4.1275" y="3.175" size="1.778" layer="95">&gt;Name</text>
<text x="-0.3175" y="1.905" size="0.254" layer="100">PaJa</text>
<pin name="1" x="-7.62" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="7.62" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="D-SCHOT">
<wire x1="-0.1588" y1="1.27" x2="-0.1588" y2="0" width="0.254" layer="94"/>
<wire x1="-0.1588" y1="0" x2="-0.1588" y2="-1.27" width="0.254" layer="94"/>
<wire x1="-2.3813" y1="-1.27" x2="-2.3813" y2="1.27" width="0.254" layer="94"/>
<wire x1="-2.3813" y1="1.27" x2="-0.1588" y2="0" width="0.254" layer="94"/>
<wire x1="-0.1588" y1="0" x2="-2.3813" y2="-1.27" width="0.254" layer="94"/>
<wire x1="-0.1588" y1="1.27" x2="-0.635" y2="1.27" width="0.254" layer="94"/>
<wire x1="-0.635" y1="1.27" x2="-0.635" y2="0.9525" width="0.254" layer="94"/>
<wire x1="-0.1588" y1="-1.27" x2="0.3175" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0.3175" y1="-1.27" x2="0.3175" y2="-0.9525" width="0.254" layer="94"/>
<text x="-2.2227" y="0.4759" size="0.254" layer="100" rot="R270">PaJa</text>
<text x="-2.5401" y="-1.905" size="1.6764" layer="96" rot="MR180">&gt;Value</text>
<text x="-2.54" y="1.905" size="1.6764" layer="95">&gt;Part</text>
<pin name="A" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
<pin name="K" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="LM317">
<wire x1="-7.62" y1="2.54" x2="-7.62" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-5.08" x2="7.62" y2="-5.08" width="0.254" layer="94"/>
<wire x1="7.62" y1="-5.08" x2="7.62" y2="2.54" width="0.254" layer="94"/>
<wire x1="7.62" y1="2.54" x2="-7.62" y2="2.54" width="0.254" layer="94"/>
<text x="3.8883" y="-0.685" size="1.27" layer="94">Out</text>
<text x="-6.8587" y="-0.685" size="1.27" layer="94">In</text>
<text x="-2.84" y="-2.3095" size="1.778" layer="95">&gt;Name</text>
<text x="-4.3405" y="0.337" size="1.778" layer="96">&gt;Value</text>
<text x="-3.2315" y="-4.3927" size="1.27" layer="94">Adjust</text>
<text x="6.35" y="-4.7625" size="0.254" layer="100">PaJa</text>
<pin name="OUT" x="10.16" y="0" visible="pad" length="short" direction="out" rot="R180"/>
<pin name="IN" x="-10.16" y="0" visible="pad" length="short" direction="in"/>
<pin name="ADJ" x="0" y="-7.62" visible="pad" length="short" direction="in" rot="R90"/>
</symbol>
<symbol name="LED">
<wire x1="0.95" y1="1.9085" x2="1.745" y2="2.7035" width="0.155" layer="94"/>
<wire x1="1.745" y1="2.7035" x2="1.268" y2="2.7035" width="0.155" layer="94"/>
<wire x1="1.745" y1="2.7035" x2="1.745" y2="2.2265" width="0.155" layer="94"/>
<wire x1="2.699" y1="2.2265" x2="2.222" y2="2.2265" width="0.155" layer="94"/>
<wire x1="2.699" y1="2.2265" x2="2.699" y2="1.7495" width="0.155" layer="94"/>
<wire x1="1.904" y1="1.4315" x2="2.699" y2="2.2265" width="0.155" layer="94"/>
<wire x1="2.3812" y1="1.27" x2="2.3812" y2="0" width="0.254" layer="94"/>
<wire x1="2.3812" y1="0" x2="2.3812" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0.1587" y1="-1.27" x2="0.1587" y2="1.27" width="0.254" layer="94"/>
<wire x1="0.1587" y1="1.27" x2="2.3812" y2="0" width="0.254" layer="94"/>
<wire x1="2.3812" y1="0" x2="0.1587" y2="-1.27" width="0.254" layer="94"/>
<text x="2.8575" y="-2.2224" size="1.6764" layer="96">&gt;Value</text>
<text x="0.3173" y="0.4759" size="0.254" layer="100" rot="R270">PaJa</text>
<text x="3.0163" y="0.4762" size="1.6764" layer="95">&gt;Part</text>
<pin name="A" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
<pin name="K" x="5.08" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="C">
<wire x1="-2.54" y1="0" x2="-2.0638" y2="0" width="0.152" layer="94"/>
<wire x1="-0.4763" y1="0" x2="0" y2="0" width="0.152" layer="94"/>
<text x="-1.111" y="-0.479" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="0.3175" y="0.635" size="1.6764" layer="95">&gt;Name</text>
<text x="0.3175" y="-0.635" size="1.6764" layer="96" rot="MR180">&gt;Value</text>
<rectangle x1="-2.2225" y1="-1.905" x2="-1.5875" y2="1.905" layer="94"/>
<rectangle x1="-0.9525" y1="-1.905" x2="-0.3175" y2="1.905" layer="94"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="2.54" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="R">
<wire x1="-2.54" y1="1.0319" x2="2.54" y2="1.0319" width="0.254" layer="94"/>
<wire x1="2.54" y1="1.0319" x2="2.54" y2="-1.0319" width="0.254" layer="94"/>
<wire x1="2.54" y1="-1.0319" x2="-2.54" y2="-1.0319" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-1.0319" x2="-2.54" y2="1.0319" width="0.254" layer="94"/>
<text x="2.3815" y="-0.476" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="-2.2225" y="1.5875" size="1.6764" layer="95">&gt;Name</text>
<text x="-2.2225" y="-1.5875" size="1.6764" layer="96" rot="MR180">&gt;Value</text>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="S1G2">
<wire x1="-1.905" y1="1.27" x2="-1.905" y2="5.715" width="0.4064" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="0" y1="3.81" x2="0" y2="2.54" width="0.6096" layer="94"/>
<wire x1="4.445" y1="5.715" x2="-1.905" y2="5.715" width="0.4064" layer="94"/>
<wire x1="-1.905" y1="1.27" x2="4.445" y2="1.27" width="0.4064" layer="94"/>
<wire x1="4.445" y1="5.715" x2="4.445" y2="1.27" width="0.4064" layer="94"/>
<text x="6.8262" y="0.1587" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<text x="-2.54" y="0.4762" size="1.778" layer="95" rot="R90">&gt;Part</text>
<text x="4.1275" y="1.5875" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="1" x="0" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="2" x="2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="S1G3">
<wire x1="-4.445" y1="1.27" x2="-4.445" y2="5.715" width="0.4064" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="0" y1="3.81" x2="0" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="3.81" x2="-2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="4.445" y1="5.715" x2="-4.445" y2="5.715" width="0.4064" layer="94"/>
<wire x1="-4.445" y1="1.27" x2="4.445" y2="1.27" width="0.4064" layer="94"/>
<wire x1="4.445" y1="5.715" x2="4.445" y2="1.27" width="0.4064" layer="94"/>
<text x="6.8262" y="0.1587" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<text x="-5.08" y="0.4762" size="1.778" layer="95" rot="R90">&gt;Part</text>
<text x="4.1275" y="1.5875" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="1" x="-2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="2" x="0" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="3" x="2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="PAD+">
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.152" layer="101"/>
<wire x1="-1.905" y1="0" x2="-0.6351" y2="0" width="0.152" layer="101"/>
<wire x1="-0.3175" y1="0" x2="0" y2="0" width="0.152" layer="94"/>
<circle x="-1.27" y="0" radius="0.9525" width="0.152" layer="94"/>
<text x="0.8068" y="-0.1588" size="0.254" layer="100" ratio="7" rot="R180">PaJa</text>
<text x="-0.159" y="0.4764" size="1.27" layer="95">&gt;Part</text>
<pin name="+" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="PAD-">
<wire x1="-1.905" y1="0" x2="-0.6351" y2="0" width="0.152" layer="101"/>
<wire x1="-0.3175" y1="0" x2="0" y2="0" width="0.152" layer="94"/>
<circle x="-1.27" y="0" radius="0.9525" width="0.152" layer="94"/>
<text x="0.8068" y="-0.1588" size="0.254" layer="100" ratio="7" rot="R180">PaJa</text>
<text x="-0.1589" y="0.4764" size="1.27" layer="95">&gt;Part</text>
<pin name="-" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="S1G5">
<wire x1="-6.985" y1="1.27" x2="-6.985" y2="5.715" width="0.4064" layer="94"/>
<wire x1="0" y1="3.81" x2="0" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="3.81" x2="-2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-5.08" y1="3.81" x2="-5.08" y2="2.54" width="0.6096" layer="94"/>
<wire x1="6.985" y1="5.715" x2="-6.985" y2="5.715" width="0.4064" layer="94"/>
<wire x1="-6.985" y1="1.27" x2="6.985" y2="1.27" width="0.4064" layer="94"/>
<wire x1="6.985" y1="5.715" x2="6.985" y2="1.27" width="0.4064" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="5.08" y1="3.81" x2="5.08" y2="2.54" width="0.6096" layer="94"/>
<text x="9.3662" y="0.1587" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<text x="-7.62" y="0.4762" size="1.778" layer="95" rot="R90">&gt;Part</text>
<text x="6.6675" y="1.5875" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="1" x="-5.08" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="2" x="-2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="3" x="0" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="4" x="2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="5" x="5.08" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="S1G7">
<wire x1="-9.525" y1="1.27" x2="-9.525" y2="5.715" width="0.4064" layer="94"/>
<wire x1="-2.54" y1="3.81" x2="-2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-5.08" y1="3.81" x2="-5.08" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-7.62" y1="3.81" x2="-7.62" y2="2.54" width="0.6096" layer="94"/>
<wire x1="0" y1="3.81" x2="0" y2="2.54" width="0.6096" layer="94"/>
<wire x1="5.08" y1="3.81" x2="5.08" y2="2.54" width="0.6096" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="9.525" y1="5.715" x2="-9.525" y2="5.715" width="0.4064" layer="94"/>
<wire x1="-9.525" y1="1.27" x2="9.525" y2="1.27" width="0.4064" layer="94"/>
<wire x1="9.525" y1="5.715" x2="9.525" y2="1.27" width="0.4064" layer="94"/>
<wire x1="7.62" y1="3.81" x2="7.62" y2="2.54" width="0.6096" layer="94"/>
<text x="-10.16" y="0.4762" size="1.778" layer="95" rot="R90">&gt;Part</text>
<text x="11.9062" y="0.1587" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<text x="9.2075" y="1.5875" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="1" x="-7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="2" x="-5.08" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="3" x="-2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="4" x="0" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="5" x="2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="6" x="5.08" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="7" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="S1G4">
<wire x1="-4.445" y1="1.27" x2="-4.445" y2="5.715" width="0.4064" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="0" y1="3.81" x2="0" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="3.81" x2="-2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="6.985" y1="5.715" x2="-4.445" y2="5.715" width="0.4064" layer="94"/>
<wire x1="-4.445" y1="1.27" x2="6.985" y2="1.27" width="0.4064" layer="94"/>
<wire x1="6.985" y1="5.715" x2="6.985" y2="1.27" width="0.4064" layer="94"/>
<wire x1="5.08" y1="3.81" x2="5.08" y2="2.54" width="0.6096" layer="94"/>
<text x="9.3662" y="0.1587" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<text x="-5.08" y="0.4762" size="1.778" layer="95" rot="R90">&gt;Part</text>
<text x="6.6675" y="1.5875" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="1" x="-2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="2" x="0" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="3" x="2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="4" x="5.08" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="ML10">
<wire x1="3.81" y1="-7.62" x2="-3.81" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="7.62" x2="-3.81" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-7.62" x2="3.81" y2="7.62" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="7.62" x2="3.81" y2="7.62" width="0.4064" layer="94"/>
<wire x1="1.27" y1="5.08" x2="2.54" y2="5.08" width="0.6096" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-5.08" x2="2.54" y2="-5.08" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-1.27" y2="5.08" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-1.27" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="-1.27" y2="-5.08" width="0.6096" layer="94"/>
<text x="-3.81" y="-10.16" size="1.778" layer="96">&gt;VALUE</text>
<text x="-3.81" y="8.382" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-7.3025" size="0.254" layer="100">PaJa</text>
<pin name="1" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="-7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="3" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="4" x="-7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="5" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="6" x="-7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="7" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="8" x="-7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="9" x="7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="10" x="-7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="C-ELEKTROLYT" prefix="C" uservalue="yes">
<description>&lt;b&gt;Kondenzator - elektrolyticky&lt;/b&gt;</description>
<gates>
<gate name="C" symbol="C-EL" x="0" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="_2" package="C-EL_2">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_2,5" package="C-EL_2,5">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_3,5" package="C-EL_3,5">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_5" package="C-EL_5">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_5+" package="C-EL_5+">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_7,5" package="C-EL_7,5">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_7,5+" package="C-EL7,5+">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_10" package="C-EL_10">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_10+" package="C-EL_10+">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="FED_2200" prefix="L">
<description>&lt;B&gt;Tlumivka FED&lt;/B&gt; - 2,2 mH &lt;I&gt;(pro stmivace)&lt;/I&gt;&lt;BR&gt;
2.3A, 340mOhm, max 500W  &lt;I&gt;(více na GES)&lt;/I&gt;</description>
<gates>
<gate name="FED" symbol="TLUMIVKA" x="0" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="" package="FED_2200">
<connects>
<connect gate="FED" pin="1" pad="1"/>
<connect gate="FED" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MBR760" prefix="D">
<description>&lt;B&gt;Schottkyho dioda&lt;/B&gt; - 15A, 60V</description>
<gates>
<gate name="D" symbol="D-SCHOT" x="0" y="0"/>
</gates>
<devices>
<device name="" package="TO-220AC">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="LM317T" prefix="IO">
<description>&lt;B&gt;Stabilizator&lt;/B&gt; - s nastavitelnym napetim - 1,5A</description>
<gates>
<gate name="IO" symbol="LM317" x="-38.1" y="38.1"/>
</gates>
<devices>
<device name="_STOJ" package="TO-220S">
<connects>
<connect gate="IO" pin="ADJ" pad="1"/>
<connect gate="IO" pin="IN" pad="3"/>
<connect gate="IO" pin="OUT" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_LEZ" package="TO-220">
<connects>
<connect gate="IO" pin="ADJ" pad="1"/>
<connect gate="IO" pin="IN" pad="3"/>
<connect gate="IO" pin="OUT" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="LED" prefix="D">
<description>&lt;B&gt;LED&lt;/B&gt; - jednobarevna</description>
<gates>
<gate name="D" symbol="LED" x="-2.54" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="_10" package="LED_10">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_3" package="LED_3">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_5" package="LED_5">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_5X5" package="LED_5X5">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_8" package="LED_8">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_TROJ" package="LED_TROJ">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_2,5X5" package="LED2,5X5">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_1206" package="P1206">
<connects>
<connect gate="D" pin="A" pad="A"/>
<connect gate="D" pin="K" pad="K"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="C-KERAMIK" prefix="C" uservalue="yes">
<description>&lt;b&gt;Kondenzator - keramicky&lt;/b&gt;</description>
<gates>
<gate name="C" symbol="C" x="0" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="_2,5" package="C-2,5">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_5" package="C-5">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_7,5" package="C-7,5">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_10" package="C-10">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_1206" package="1206">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_0805" package="0805">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="R" prefix="R" uservalue="yes">
<description>&lt;b&gt;Rezistor&lt;/b&gt;</description>
<gates>
<gate name="R" symbol="R" x="0" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="_5" package="R-5">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_10" package="R-10">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_12,7" package="R-12,7">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_7,5" package="R-7,5">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_1206" package="1206">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="__2W" package="R-_2W">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="__10W" package="R-_10W">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="__20W" package="R-_20W">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="__5W" package="R-_5W">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_STOJ" package="R-STOJ">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_0805" package="0805">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="S1G2_JUMP" prefix="JUM">
<description>&lt;B&gt;Radove konektory&lt;/B&gt; - koliky - 2x</description>
<gates>
<gate name="JUMP" symbol="S1G2" x="-40.64" y="40.64"/>
</gates>
<devices>
<device name="" package="S1G2_JUM">
<connects>
<connect gate="JUMP" pin="1" pad="1"/>
<connect gate="JUMP" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="S1G3_JUMP" prefix="JUM">
<description>&lt;B&gt;Radove konektory&lt;/B&gt; - koliky - 3x</description>
<gates>
<gate name="JUMP" symbol="S1G3" x="-43.18" y="33.02"/>
</gates>
<devices>
<device name="" package="S1G3_JUM">
<connects>
<connect gate="JUMP" pin="1" pad="1"/>
<connect gate="JUMP" pin="2" pad="2"/>
<connect gate="JUMP" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="PAD_1+" prefix="PAD">
<description>&lt;B&gt;Pajeci bod&lt;/B&gt; - 2,54 mm prumer, + kladny</description>
<gates>
<gate name="PAD" symbol="PAD+" x="0" y="0"/>
</gates>
<devices>
<device name="" package="PAD_1">
<connects>
<connect gate="PAD" pin="+" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="PAD_1-" prefix="PAD">
<description>&lt;B&gt;Pajeci bod&lt;/B&gt; - 2,54 mm prumer, - zaporny</description>
<gates>
<gate name="PAD" symbol="PAD-" x="0" y="0"/>
</gates>
<devices>
<device name="" package="PAD_1">
<connects>
<connect gate="PAD" pin="-" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="S1G5_JUMP" prefix="JUM">
<description>&lt;B&gt;Radove konektory&lt;/B&gt; - koliky - 5x</description>
<gates>
<gate name="JUMP" symbol="S1G5" x="-40.64" y="30.48"/>
</gates>
<devices>
<device name="" package="S1G5_JUM">
<connects>
<connect gate="JUMP" pin="1" pad="1"/>
<connect gate="JUMP" pin="2" pad="2"/>
<connect gate="JUMP" pin="3" pad="3"/>
<connect gate="JUMP" pin="4" pad="4"/>
<connect gate="JUMP" pin="5" pad="5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="S1G7_JUMP" prefix="JUM">
<description>&lt;B&gt;Radove konektory&lt;/B&gt; - koliky - 7x</description>
<gates>
<gate name="JUM" symbol="S1G7" x="0" y="0"/>
</gates>
<devices>
<device name="" package="S1G7_JUM">
<connects>
<connect gate="JUM" pin="1" pad="1"/>
<connect gate="JUM" pin="2" pad="2"/>
<connect gate="JUM" pin="3" pad="3"/>
<connect gate="JUM" pin="4" pad="4"/>
<connect gate="JUM" pin="5" pad="5"/>
<connect gate="JUM" pin="6" pad="6"/>
<connect gate="JUM" pin="7" pad="7"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="S1G4_JUMP" prefix="JUM">
<description>&lt;B&gt;Radove konektory&lt;/B&gt; - koliky - 4x</description>
<gates>
<gate name="JUMP" symbol="S1G4" x="-40.64" y="33.02"/>
</gates>
<devices>
<device name="" package="S1G4_JUM">
<connects>
<connect gate="JUMP" pin="1" pad="1"/>
<connect gate="JUMP" pin="2" pad="2"/>
<connect gate="JUMP" pin="3" pad="3"/>
<connect gate="JUMP" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MLW10" prefix="CON">
<description>&lt;B&gt;Konektory MLW&lt;/B&gt; - vidlice - 10x</description>
<gates>
<gate name="CON" symbol="ML10" x="-40.64" y="30.48" swaplevel="1"/>
</gates>
<devices>
<device name="_90°" package="MLW10A">
<connects>
<connect gate="CON" pin="1" pad="1"/>
<connect gate="CON" pin="10" pad="10"/>
<connect gate="CON" pin="2" pad="2"/>
<connect gate="CON" pin="3" pad="3"/>
<connect gate="CON" pin="4" pad="4"/>
<connect gate="CON" pin="5" pad="5"/>
<connect gate="CON" pin="6" pad="6"/>
<connect gate="CON" pin="7" pad="7"/>
<connect gate="CON" pin="8" pad="8"/>
<connect gate="CON" pin="9" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="" package="MLW10G">
<connects>
<connect gate="CON" pin="1" pad="1"/>
<connect gate="CON" pin="10" pad="10"/>
<connect gate="CON" pin="2" pad="2"/>
<connect gate="CON" pin="3" pad="3"/>
<connect gate="CON" pin="4" pad="4"/>
<connect gate="CON" pin="5" pad="5"/>
<connect gate="CON" pin="6" pad="6"/>
<connect gate="CON" pin="7" pad="7"/>
<connect gate="CON" pin="8" pad="8"/>
<connect gate="CON" pin="9" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="LM2576">
<packages>
<package name="LM2576">
<wire x1="-5.08" y1="-2.54" x2="-5.08" y2="2.54" width="0.127" layer="21"/>
<wire x1="-5.08" y1="2.54" x2="5.08" y2="2.54" width="0.127" layer="21"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-2.54" width="0.127" layer="21"/>
<wire x1="5.08" y1="-2.54" x2="-5.08" y2="-2.54" width="0.127" layer="21"/>
<pad name="1" x="-3.556" y="0" drill="1.016"/>
<pad name="2" x="-1.778" y="0" drill="1.016"/>
<pad name="3" x="0" y="0" drill="1.016"/>
<pad name="4" x="1.778" y="0" drill="1.016"/>
<pad name="5" x="3.556" y="0" drill="1.016"/>
</package>
</packages>
<symbols>
<symbol name="LM2576">
<wire x1="-7.62" y1="7.62" x2="-7.62" y2="-12.7" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-12.7" x2="5.08" y2="-12.7" width="0.254" layer="94"/>
<wire x1="5.08" y1="-12.7" x2="5.08" y2="7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="7.62" x2="-7.62" y2="7.62" width="0.254" layer="94"/>
<text x="-5.08" y="5.08" size="1.778" layer="96">&gt;value</text>
<text x="-5.08" y="10.16" size="1.778" layer="95">&gt;name</text>
<pin name="IN" x="-12.7" y="2.54" length="middle"/>
<pin name="OUT" x="10.16" y="0" length="middle" rot="R180"/>
<pin name="GND" x="-5.08" y="-17.78" length="middle" rot="R90"/>
<pin name="FB" x="10.16" y="2.54" length="middle" rot="R180"/>
<pin name="ON*/OFF" x="2.54" y="-17.78" length="middle" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="LM2576">
<description>LM2576 - 5V switching regulator</description>
<gates>
<gate name="G$1" symbol="LM2576" x="2.54" y="-2.54"/>
</gates>
<devices>
<device name="" package="LM2576">
<connects>
<connect gate="G$1" pin="FB" pad="4"/>
<connect gate="G$1" pin="GND" pad="5"/>
<connect gate="G$1" pin="IN" pad="1"/>
<connect gate="G$1" pin="ON*/OFF" pad="3"/>
<connect gate="G$1" pin="OUT" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="dodolcd">
<description>DODOTRONIC personal library free use
Italy
www. dodotronic.com</description>
<packages>
<package name="PN61729">
<description>&lt;b&gt;BERG&lt;/b&gt;</description>
<wire x1="-5.9" y1="5.6" x2="-5.9" y2="-10.15" width="0.254" layer="21"/>
<wire x1="-5.9" y1="-10.15" x2="5.9" y2="-10.15" width="0.254" layer="21"/>
<wire x1="5.9" y1="-10.15" x2="5.9" y2="5.6" width="0.254" layer="21"/>
<wire x1="5.9" y1="5.6" x2="-5.9" y2="5.6" width="0.254" layer="21"/>
<pad name="1" x="1.25" y="4.71" drill="0.95" shape="octagon"/>
<pad name="2" x="-1.25" y="4.71" drill="0.95" shape="octagon"/>
<pad name="3" x="-1.25" y="2.71" drill="0.95" shape="octagon"/>
<pad name="4" x="1.25" y="2.71" drill="0.95" shape="octagon"/>
<pad name="P$1" x="-6.0198" y="0" drill="2.2"/>
<pad name="P$2" x="6.0198" y="0" drill="2.2"/>
<text x="-6.35" y="6.35" size="1.27" layer="25">&gt;NAME</text>
<text x="7.62" y="-8.89" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-1.9304" y="-8.0264" size="1.27" layer="21">USB</text>
</package>
</packages>
<symbols>
<symbol name="USB">
<wire x1="2.54" y1="5.08" x2="-2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.62" x2="2.54" y2="-7.62" width="0.254" layer="94"/>
<text x="6.35" y="-5.08" size="1.27" layer="95" rot="R90">&gt;NAME</text>
<text x="-2.54" y="6.35" size="1.27" layer="96">&gt;VALUE</text>
<pin name="VCC" x="-7.62" y="2.54" length="middle" direction="sup"/>
<pin name="D-" x="-7.62" y="0" length="middle"/>
<pin name="D+" x="-7.62" y="-2.54" length="middle"/>
<pin name="GND" x="-7.62" y="-5.08" length="middle" direction="sup"/>
<pin name="P$1" x="2.54" y="-12.7" visible="off" length="middle" direction="sup" rot="R90"/>
<pin name="P$2" x="0" y="-12.7" visible="off" length="middle" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="USB" prefix="CONN">
<description>USB connector</description>
<gates>
<gate name="G$1" symbol="USB" x="0" y="0"/>
</gates>
<devices>
<device name="" package="PN61729">
<connects>
<connect gate="G$1" pin="D+" pad="3"/>
<connect gate="G$1" pin="D-" pad="2"/>
<connect gate="G$1" pin="GND" pad="4"/>
<connect gate="G$1" pin="P$1" pad="P$1"/>
<connect gate="G$1" pin="P$2" pad="P$2"/>
<connect gate="G$1" pin="VCC" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="IC1" library="atmel" deviceset="MEGA8-P" device=""/>
<part name="P+1" library="supply1" deviceset="+5V" device=""/>
<part name="GND2" library="supply1" deviceset="GND" device=""/>
<part name="GND3" library="supply1" deviceset="GND" device=""/>
<part name="GND4" library="supply1" deviceset="GND" device=""/>
<part name="C2" library="#PaJa_22" deviceset="C-ELEKTROLYT" device="_5" value="100uF"/>
<part name="GND6" library="supply1" deviceset="GND" device=""/>
<part name="GND7" library="supply1" deviceset="GND" device=""/>
<part name="GND8" library="supply1" deviceset="GND" device=""/>
<part name="GND9" library="supply1" deviceset="GND" device=""/>
<part name="L1" library="#PaJa_22" deviceset="FED_2200" device="" value="100uH"/>
<part name="C1" library="#PaJa_22" deviceset="C-ELEKTROLYT" device="_5" value="1000uF"/>
<part name="D1" library="#PaJa_22" deviceset="MBR760" device="" value="1N5822"/>
<part name="IO1" library="#PaJa_22" deviceset="LM317T" device="_LEZ" value="LM3940IT"/>
<part name="C3" library="#PaJa_22" deviceset="C-ELEKTROLYT" device="_5" value="0,47uF"/>
<part name="C4" library="#PaJa_22" deviceset="C-ELEKTROLYT" device="_3,5" value="0,33uF"/>
<part name="C6" library="#PaJa_22" deviceset="C-KERAMIK" device="_2,5" value="100n"/>
<part name="C7" library="#PaJa_22" deviceset="C-KERAMIK" device="_2,5" value="100n"/>
<part name="C8" library="#PaJa_22" deviceset="C-KERAMIK" device="_2,5" value="100n"/>
<part name="R1" library="#PaJa_22" deviceset="R" device="_10" value="10k"/>
<part name="JUM5" library="#PaJa_22" deviceset="S1G2_JUMP" device="" value="motor_6V_out"/>
<part name="GND1" library="supply1" deviceset="GND" device=""/>
<part name="GND11" library="supply1" deviceset="GND" device=""/>
<part name="JUM4" library="#PaJa_22" deviceset="S1G3_JUMP" device="" value="regulator_vstup"/>
<part name="R2" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="GND12" library="supply1" deviceset="GND" device=""/>
<part name="D3" library="#PaJa_22" deviceset="LED" device="_3"/>
<part name="PAD5" library="#PaJa_22" deviceset="PAD_1+" device="" value="11,1V"/>
<part name="PAD6" library="#PaJa_22" deviceset="PAD_1-" device=""/>
<part name="R3" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="D2" library="#PaJa_22" deviceset="LED" device="_3"/>
<part name="PAD3" library="#PaJa_22" deviceset="PAD_1+" device="" value="motor_6V_in"/>
<part name="PAD7" library="#PaJa_22" deviceset="PAD_1-" device="" value="6V_motor"/>
<part name="JUM6" library="#PaJa_22" deviceset="S1G2_JUMP" device="" value="napajeni_Maestro"/>
<part name="JUM9" library="#PaJa_22" deviceset="S1G2_JUMP" device=""/>
<part name="JUM11" library="#PaJa_22" deviceset="S1G5_JUMP" device=""/>
<part name="JUM12" library="#PaJa_22" deviceset="S1G7_JUMP" device=""/>
<part name="D4" library="#PaJa_22" deviceset="LED" device="_3"/>
<part name="D5" library="#PaJa_22" deviceset="LED" device="_3"/>
<part name="R4" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="R5" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="GND5" library="supply1" deviceset="GND" device=""/>
<part name="GND13" library="supply1" deviceset="GND" device=""/>
<part name="JUM1" library="#PaJa_22" deviceset="S1G4_JUMP" device="" value="senzory_ir_reflex"/>
<part name="JUM3" library="#PaJa_22" deviceset="S1G4_JUMP" device="" value="senzory_ir_reflex_GND"/>
<part name="GND14" library="supply1" deviceset="GND" device=""/>
<part name="R6" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="R7" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="R8" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="R10" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="R11" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="R12" library="#PaJa_22" deviceset="R" device="_10" value="120"/>
<part name="JUM13" library="#PaJa_22" deviceset="S1G2_JUMP" device="" value="jumper"/>
<part name="JUM14" library="#PaJa_22" deviceset="S1G2_JUMP" device="" value="jumper1"/>
<part name="JUM15" library="#PaJa_22" deviceset="S1G4_JUMP" device="" value="senzor_vzdal_signal_in"/>
<part name="JUM8" library="#PaJa_22" deviceset="S1G4_JUMP" device="" value="senzor_vzdal_-"/>
<part name="JUM17" library="#PaJa_22" deviceset="S1G4_JUMP" device="" value="senzor_vzdal_+"/>
<part name="JUM7" library="#PaJa_22" deviceset="S1G4_JUMP" device="" value="piny_+5V"/>
<part name="JUM18" library="#PaJa_22" deviceset="S1G4_JUMP" device="" value="piny_-5V"/>
<part name="GND10" library="supply1" deviceset="GND" device=""/>
<part name="JUM2" library="#PaJa_22" deviceset="S1G4_JUMP" device="" value="senzory_ir_reflex_signal"/>
<part name="JUM19" library="#PaJa_22" deviceset="S1G2_JUMP" device="" value="I2C"/>
<part name="JUM20" library="#PaJa_22" deviceset="S1G5_JUMP" device="" value="out_vzdal_senzor_signal"/>
<part name="CON1" library="#PaJa_22" deviceset="MLW10" device="_90°" value="ISP"/>
<part name="GND15" library="supply1" deviceset="GND" device=""/>
<part name="GND16" library="supply1" deviceset="GND" device=""/>
<part name="R9" library="#PaJa_22" deviceset="R" device="_10" value="10uH"/>
<part name="U$1" library="LM2576" deviceset="LM2576" device=""/>
<part name="CONN1" library="dodolcd" deviceset="USB" device=""/>
<part name="GND17" library="supply1" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="IC1" gate="G$1" x="205.74" y="134.62"/>
<instance part="P+1" gate="1" x="172.72" y="63.5" rot="R270"/>
<instance part="GND2" gate="1" x="58.42" y="30.48"/>
<instance part="GND3" gate="1" x="83.82" y="121.92"/>
<instance part="GND4" gate="1" x="139.7" y="167.64" rot="R180"/>
<instance part="C2" gate="C" x="58.42" y="45.72" rot="R270"/>
<instance part="GND6" gate="1" x="78.74" y="30.48"/>
<instance part="GND7" gate="1" x="86.36" y="30.48"/>
<instance part="GND8" gate="1" x="111.76" y="30.48"/>
<instance part="GND9" gate="1" x="127" y="30.48"/>
<instance part="L1" gate="FED" x="119.38" y="63.5"/>
<instance part="C1" gate="C" x="127" y="45.72" rot="R270"/>
<instance part="D1" gate="D" x="111.76" y="48.26" rot="R90"/>
<instance part="IO1" gate="IO" x="83.82" y="142.24"/>
<instance part="C3" gate="C" x="63.5" y="129.54" rot="R270"/>
<instance part="C4" gate="C" x="104.14" y="129.54" rot="R270"/>
<instance part="C6" gate="C" x="170.18" y="154.94" rot="R180"/>
<instance part="C7" gate="C" x="149.86" y="124.46" rot="R90"/>
<instance part="C8" gate="C" x="152.4" y="152.4" rot="R180"/>
<instance part="R1" gate="R" x="129.54" y="127" rot="R90"/>
<instance part="JUM5" gate="JUMP" x="203.2" y="66.04"/>
<instance part="GND1" gate="1" x="48.26" y="114.3"/>
<instance part="GND11" gate="1" x="177.8" y="33.02"/>
<instance part="JUM4" gate="JUMP" x="40.64" y="142.24" rot="R90"/>
<instance part="R2" gate="R" x="55.88" y="137.16" rot="R90"/>
<instance part="GND12" gate="1" x="55.88" y="114.3"/>
<instance part="D3" gate="D" x="55.88" y="124.46" rot="R270"/>
<instance part="PAD5" gate="PAD" x="45.72" y="68.58"/>
<instance part="PAD6" gate="PAD" x="45.72" y="33.02"/>
<instance part="R3" gate="R" x="134.62" y="58.42" rot="R90"/>
<instance part="D2" gate="D" x="134.62" y="45.72" rot="R270"/>
<instance part="PAD3" gate="PAD" x="243.84" y="68.58" rot="R270"/>
<instance part="PAD7" gate="PAD" x="248.92" y="68.58" rot="R270"/>
<instance part="JUM6" gate="JUMP" x="45.72" y="48.26" rot="R90"/>
<instance part="JUM9" gate="JUMP" x="170.18" y="142.24" rot="R90"/>
<instance part="JUM11" gate="JUMP" x="261.62" y="111.76" rot="R270"/>
<instance part="JUM12" gate="JUM" x="261.62" y="132.08" rot="R270"/>
<instance part="D4" gate="D" x="294.64" y="121.92"/>
<instance part="D5" gate="D" x="294.64" y="104.14"/>
<instance part="R4" gate="R" x="284.48" y="104.14" rot="R180"/>
<instance part="R5" gate="R" x="284.48" y="121.92" rot="R180"/>
<instance part="GND5" gate="1" x="304.8" y="104.14" rot="R90"/>
<instance part="GND13" gate="1" x="304.8" y="121.92" rot="R90"/>
<instance part="JUM1" gate="JUMP" x="111.76" y="208.28"/>
<instance part="JUM3" gate="JUMP" x="76.2" y="208.28"/>
<instance part="GND14" gate="1" x="88.9" y="205.74" rot="R90"/>
<instance part="R6" gate="R" x="119.38" y="198.12" rot="R90"/>
<instance part="R7" gate="R" x="111.76" y="198.12" rot="R90"/>
<instance part="R8" gate="R" x="127" y="200.66" rot="R90"/>
<instance part="R10" gate="R" x="165.1" y="177.8" rot="R180"/>
<instance part="R11" gate="R" x="180.34" y="182.88" rot="R180"/>
<instance part="R12" gate="R" x="193.04" y="187.96" rot="R180"/>
<instance part="JUM13" gate="JUMP" x="274.32" y="124.46"/>
<instance part="JUM14" gate="JUMP" x="276.86" y="101.6" rot="R180"/>
<instance part="JUM15" gate="JUMP" x="157.48" y="50.8"/>
<instance part="JUM8" gate="JUMP" x="160.02" y="33.02" rot="R180"/>
<instance part="JUM17" gate="JUMP" x="157.48" y="66.04"/>
<instance part="JUM7" gate="JUMP" x="139.7" y="66.04"/>
<instance part="JUM18" gate="JUMP" x="142.24" y="33.02" rot="R180"/>
<instance part="GND10" gate="1" x="35.56" y="109.22" rot="R180"/>
<instance part="JUM2" gate="JUMP" x="261.62" y="160.02" rot="R270"/>
<instance part="JUM19" gate="JUMP" x="294.64" y="149.86" rot="R270"/>
<instance part="JUM20" gate="JUMP" x="195.58" y="33.02" rot="R180"/>
<instance part="CON1" gate="CON" x="304.8" y="203.2" rot="R180"/>
<instance part="GND15" gate="1" x="322.58" y="198.12" rot="R90"/>
<instance part="GND16" gate="1" x="292.1" y="205.74" rot="R270"/>
<instance part="R9" gate="R" x="160.02" y="124.46" rot="R90"/>
<instance part="U$1" gate="G$1" x="83.82" y="73.66"/>
<instance part="CONN1" gate="G$1" x="27.94" y="91.44" rot="R180"/>
<instance part="GND17" gate="1" x="25.4" y="106.68" rot="R180"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="C2" gate="C" pin="C_EL-"/>
<pinref part="GND2" gate="1" pin="GND"/>
<wire x1="58.42" y1="33.02" x2="58.42" y2="43.18" width="0.1524" layer="91"/>
<junction x="58.42" y="33.02"/>
<pinref part="PAD6" gate="PAD" pin="-"/>
<wire x1="48.26" y1="33.02" x2="53.34" y2="33.02" width="0.1524" layer="91"/>
<pinref part="JUM6" gate="JUMP" pin="1"/>
<wire x1="53.34" y1="33.02" x2="58.42" y2="33.02" width="0.1524" layer="91"/>
<wire x1="48.26" y1="48.26" x2="53.34" y2="48.26" width="0.1524" layer="91"/>
<wire x1="53.34" y1="48.26" x2="53.34" y2="33.02" width="0.1524" layer="91"/>
<junction x="53.34" y="33.02"/>
</segment>
<segment>
<pinref part="GND6" gate="1" pin="GND"/>
<wire x1="78.74" y1="33.02" x2="78.74" y2="55.88" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="GND"/>
</segment>
<segment>
<pinref part="GND7" gate="1" pin="GND"/>
<wire x1="86.36" y1="33.02" x2="86.36" y2="55.88" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="ON*/OFF"/>
</segment>
<segment>
<pinref part="GND8" gate="1" pin="GND"/>
<wire x1="111.76" y1="43.18" x2="111.76" y2="33.02" width="0.1524" layer="91"/>
<pinref part="D1" gate="D" pin="A"/>
</segment>
<segment>
<pinref part="C1" gate="C" pin="C_EL-"/>
<pinref part="GND9" gate="1" pin="GND"/>
<wire x1="127" y1="33.02" x2="127" y2="35.56" width="0.1524" layer="91"/>
<wire x1="127" y1="35.56" x2="127" y2="43.18" width="0.1524" layer="91"/>
<wire x1="127" y1="35.56" x2="134.62" y2="35.56" width="0.1524" layer="91"/>
<wire x1="134.62" y1="35.56" x2="137.16" y2="35.56" width="0.1524" layer="91"/>
<junction x="127" y="35.56"/>
<pinref part="D2" gate="D" pin="K"/>
<wire x1="137.16" y1="35.56" x2="139.7" y2="35.56" width="0.1524" layer="91"/>
<wire x1="139.7" y1="35.56" x2="142.24" y2="35.56" width="0.1524" layer="91"/>
<wire x1="142.24" y1="35.56" x2="144.78" y2="35.56" width="0.1524" layer="91"/>
<wire x1="134.62" y1="40.64" x2="134.62" y2="35.56" width="0.1524" layer="91"/>
<junction x="134.62" y="35.56"/>
<pinref part="JUM18" gate="JUMP" pin="1"/>
<pinref part="JUM18" gate="JUMP" pin="2"/>
<pinref part="JUM18" gate="JUMP" pin="3"/>
<pinref part="JUM18" gate="JUMP" pin="4"/>
<pinref part="GND11" gate="1" pin="GND"/>
<wire x1="162.56" y1="35.56" x2="177.8" y2="35.56" width="0.1524" layer="91"/>
<junction x="162.56" y="35.56"/>
<pinref part="JUM8" gate="JUMP" pin="1"/>
<wire x1="160.02" y1="35.56" x2="162.56" y2="35.56" width="0.1524" layer="91"/>
<junction x="160.02" y="35.56"/>
<pinref part="JUM8" gate="JUMP" pin="2"/>
<wire x1="157.48" y1="35.56" x2="160.02" y2="35.56" width="0.1524" layer="91"/>
<junction x="157.48" y="35.56"/>
<pinref part="JUM8" gate="JUMP" pin="3"/>
<wire x1="154.94" y1="35.56" x2="157.48" y2="35.56" width="0.1524" layer="91"/>
<junction x="154.94" y="35.56"/>
<pinref part="JUM8" gate="JUMP" pin="4"/>
<wire x1="144.78" y1="35.56" x2="154.94" y2="35.56" width="0.1524" layer="91"/>
<junction x="137.16" y="35.56"/>
<junction x="139.7" y="35.56"/>
<junction x="142.24" y="35.56"/>
<junction x="144.78" y="35.56"/>
</segment>
<segment>
<pinref part="IO1" gate="IO" pin="ADJ"/>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="83.82" y1="124.46" x2="83.82" y2="134.62" width="0.1524" layer="91"/>
<pinref part="C3" gate="C" pin="C_EL-"/>
<wire x1="63.5" y1="127" x2="63.5" y2="124.46" width="0.1524" layer="91"/>
<wire x1="63.5" y1="124.46" x2="83.82" y2="124.46" width="0.1524" layer="91"/>
<pinref part="C4" gate="C" pin="C_EL-"/>
<wire x1="83.82" y1="124.46" x2="104.14" y2="124.46" width="0.1524" layer="91"/>
<wire x1="104.14" y1="124.46" x2="104.14" y2="127" width="0.1524" layer="91"/>
<junction x="83.82" y="124.46"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND@1"/>
<wire x1="139.7" y1="132.08" x2="149.86" y2="132.08" width="0.1524" layer="91"/>
<wire x1="149.86" y1="132.08" x2="182.88" y2="132.08" width="0.1524" layer="91"/>
<wire x1="139.7" y1="132.08" x2="139.7" y2="157.48" width="0.1524" layer="91"/>
<pinref part="C7" gate="C" pin="2"/>
<wire x1="149.86" y1="132.08" x2="149.86" y2="127" width="0.1524" layer="91"/>
<junction x="149.86" y="132.08"/>
<pinref part="C8" gate="C" pin="2"/>
<pinref part="IC1" gate="G$1" pin="GND"/>
<wire x1="149.86" y1="157.48" x2="182.88" y2="157.48" width="0.1524" layer="91"/>
<pinref part="C6" gate="C" pin="2"/>
<wire x1="167.64" y1="154.94" x2="149.86" y2="154.94" width="0.1524" layer="91"/>
<wire x1="149.86" y1="154.94" x2="149.86" y2="157.48" width="0.1524" layer="91"/>
<junction x="149.86" y="157.48"/>
<wire x1="142.24" y1="157.48" x2="149.86" y2="157.48" width="0.1524" layer="91"/>
<junction x="142.24" y="157.48"/>
<wire x1="142.24" y1="157.48" x2="142.24" y2="152.4" width="0.1524" layer="91"/>
<wire x1="142.24" y1="152.4" x2="149.86" y2="152.4" width="0.1524" layer="91"/>
<wire x1="142.24" y1="157.48" x2="139.7" y2="157.48" width="0.1524" layer="91"/>
<junction x="139.7" y="157.48"/>
<pinref part="GND4" gate="1" pin="GND"/>
<wire x1="139.7" y1="157.48" x2="139.7" y2="165.1" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="48.26" y1="121.92" x2="48.26" y2="116.84" width="0.1524" layer="91"/>
<wire x1="35.56" y1="99.06" x2="43.18" y2="99.06" width="0.1524" layer="91"/>
<wire x1="43.18" y1="99.06" x2="43.18" y2="121.92" width="0.1524" layer="91"/>
<wire x1="43.18" y1="121.92" x2="48.26" y2="121.92" width="0.1524" layer="91"/>
<junction x="48.26" y="121.92"/>
<wire x1="48.26" y1="121.92" x2="48.26" y2="139.7" width="0.1524" layer="91"/>
<pinref part="JUM4" gate="JUMP" pin="1"/>
<wire x1="43.18" y1="139.7" x2="48.26" y2="139.7" width="0.1524" layer="91"/>
<wire x1="35.56" y1="91.44" x2="35.56" y2="93.98" width="0.1524" layer="91"/>
<pinref part="GND10" gate="1" pin="GND"/>
<junction x="35.56" y="96.52"/>
<wire x1="35.56" y1="93.98" x2="35.56" y2="96.52" width="0.1524" layer="91"/>
<wire x1="35.56" y1="96.52" x2="35.56" y2="99.06" width="0.1524" layer="91"/>
<junction x="35.56" y="93.98"/>
<junction x="35.56" y="91.44"/>
<junction x="35.56" y="99.06"/>
<wire x1="35.56" y1="99.06" x2="35.56" y2="106.68" width="0.1524" layer="91"/>
<pinref part="CONN1" gate="G$1" pin="D-"/>
<pinref part="CONN1" gate="G$1" pin="D+"/>
<pinref part="CONN1" gate="G$1" pin="GND"/>
</segment>
<segment>
<pinref part="GND12" gate="1" pin="GND"/>
<wire x1="55.88" y1="116.84" x2="55.88" y2="119.38" width="0.1524" layer="91"/>
<pinref part="D3" gate="D" pin="K"/>
</segment>
<segment>
<pinref part="D5" gate="D" pin="K"/>
<wire x1="299.72" y1="104.14" x2="302.26" y2="104.14" width="0.1524" layer="91"/>
<pinref part="GND5" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="D4" gate="D" pin="K"/>
<wire x1="299.72" y1="121.92" x2="302.26" y2="121.92" width="0.1524" layer="91"/>
<pinref part="GND13" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="JUM3" gate="JUMP" pin="1"/>
<wire x1="73.66" y1="205.74" x2="76.2" y2="205.74" width="0.1524" layer="91"/>
<junction x="73.66" y="205.74"/>
<pinref part="JUM3" gate="JUMP" pin="2"/>
<junction x="76.2" y="205.74"/>
<wire x1="76.2" y1="205.74" x2="78.74" y2="205.74" width="0.1524" layer="91"/>
<pinref part="JUM3" gate="JUMP" pin="3"/>
<junction x="78.74" y="205.74"/>
<wire x1="78.74" y1="205.74" x2="81.28" y2="205.74" width="0.1524" layer="91"/>
<pinref part="JUM3" gate="JUMP" pin="4"/>
<junction x="81.28" y="205.74"/>
<wire x1="81.28" y1="205.74" x2="86.36" y2="205.74" width="0.1524" layer="91"/>
<pinref part="GND14" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON1" gate="CON" pin="4"/>
<pinref part="CON1" gate="CON" pin="10"/>
<wire x1="312.42" y1="205.74" x2="312.42" y2="203.2" width="0.1524" layer="91"/>
<wire x1="312.42" y1="203.2" x2="312.42" y2="200.66" width="0.1524" layer="91"/>
<wire x1="312.42" y1="200.66" x2="312.42" y2="198.12" width="0.1524" layer="91"/>
<wire x1="312.42" y1="198.12" x2="320.04" y2="198.12" width="0.1524" layer="91"/>
<pinref part="GND15" gate="1" pin="GND"/>
<pinref part="CON1" gate="CON" pin="8"/>
<junction x="312.42" y="200.66"/>
<pinref part="CON1" gate="CON" pin="6"/>
<junction x="312.42" y="203.2"/>
<junction x="312.42" y="205.74"/>
<junction x="312.42" y="198.12"/>
</segment>
<segment>
<pinref part="CON1" gate="CON" pin="3"/>
<pinref part="GND16" gate="1" pin="GND"/>
<wire x1="294.64" y1="205.74" x2="297.18" y2="205.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="CONN1" gate="G$1" pin="P$1"/>
<pinref part="GND17" gate="1" pin="GND"/>
<pinref part="CONN1" gate="G$1" pin="P$2"/>
<wire x1="25.4" y1="104.14" x2="27.94" y2="104.14" width="0.1524" layer="91"/>
<junction x="25.4" y="104.14"/>
<junction x="27.94" y="104.14"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="L1" gate="FED" pin="1"/>
<wire x1="111.76" y1="50.8" x2="111.76" y2="63.5" width="0.1524" layer="91"/>
<wire x1="93.98" y1="63.5" x2="111.76" y2="63.5" width="0.1524" layer="91"/>
<junction x="111.76" y="63.5"/>
<pinref part="D1" gate="D" pin="K"/>
<pinref part="U$1" gate="G$1" pin="OUT"/>
<wire x1="93.98" y1="63.5" x2="93.98" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="+5V" class="0">
<segment>
<pinref part="P+1" gate="1" pin="+5V"/>
<wire x1="162.56" y1="63.5" x2="170.18" y2="63.5" width="0.1524" layer="91"/>
<junction x="162.56" y="63.5"/>
<pinref part="JUM17" gate="JUMP" pin="4"/>
<wire x1="160.02" y1="63.5" x2="162.56" y2="63.5" width="0.1524" layer="91"/>
<junction x="160.02" y="63.5"/>
<pinref part="JUM17" gate="JUMP" pin="3"/>
<wire x1="157.48" y1="63.5" x2="160.02" y2="63.5" width="0.1524" layer="91"/>
<junction x="157.48" y="63.5"/>
<pinref part="JUM17" gate="JUMP" pin="2"/>
<wire x1="154.94" y1="63.5" x2="157.48" y2="63.5" width="0.1524" layer="91"/>
<junction x="154.94" y="63.5"/>
<pinref part="JUM17" gate="JUMP" pin="1"/>
<pinref part="L1" gate="FED" pin="2"/>
<wire x1="99.06" y1="73.66" x2="127" y2="73.66" width="0.1524" layer="91"/>
<wire x1="127" y1="73.66" x2="127" y2="63.5" width="0.1524" layer="91"/>
<wire x1="127" y1="63.5" x2="134.62" y2="63.5" width="0.1524" layer="91"/>
<pinref part="C1" gate="C" pin="C_EL+"/>
<wire x1="127" y1="50.8" x2="127" y2="63.5" width="0.1524" layer="91"/>
<junction x="127" y="63.5"/>
<pinref part="R3" gate="R" pin="2"/>
<junction x="134.62" y="63.5"/>
<wire x1="134.62" y1="63.5" x2="137.16" y2="63.5" width="0.1524" layer="91"/>
<pinref part="JUM7" gate="JUMP" pin="1"/>
<junction x="137.16" y="63.5"/>
<wire x1="137.16" y1="63.5" x2="139.7" y2="63.5" width="0.1524" layer="91"/>
<pinref part="JUM7" gate="JUMP" pin="2"/>
<junction x="139.7" y="63.5"/>
<wire x1="139.7" y1="63.5" x2="142.24" y2="63.5" width="0.1524" layer="91"/>
<pinref part="JUM7" gate="JUMP" pin="3"/>
<junction x="142.24" y="63.5"/>
<wire x1="142.24" y1="63.5" x2="144.78" y2="63.5" width="0.1524" layer="91"/>
<pinref part="JUM7" gate="JUMP" pin="4"/>
<junction x="144.78" y="63.5"/>
<wire x1="144.78" y1="63.5" x2="154.94" y2="63.5" width="0.1524" layer="91"/>
<wire x1="99.06" y1="73.66" x2="99.06" y2="76.2" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="FB"/>
<wire x1="99.06" y1="76.2" x2="93.98" y2="76.2" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="C6" gate="C" pin="1"/>
<pinref part="IC1" gate="G$1" pin="AREF"/>
<wire x1="175.26" y1="154.94" x2="182.88" y2="154.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="C8" gate="C" pin="1"/>
<wire x1="157.48" y1="152.4" x2="160.02" y2="152.4" width="0.1524" layer="91"/>
<wire x1="160.02" y1="129.54" x2="160.02" y2="152.4" width="0.1524" layer="91"/>
<junction x="160.02" y="152.4"/>
<pinref part="IC1" gate="G$1" pin="AVCC"/>
<wire x1="182.88" y1="152.4" x2="160.02" y2="152.4" width="0.1524" layer="91"/>
<pinref part="R9" gate="R" pin="2"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC6(/RESET)"/>
<wire x1="182.88" y1="162.56" x2="154.94" y2="162.56" width="0.1524" layer="91"/>
<wire x1="154.94" y1="162.56" x2="129.54" y2="162.56" width="0.1524" layer="91"/>
<wire x1="129.54" y1="162.56" x2="129.54" y2="132.08" width="0.1524" layer="91"/>
<pinref part="R1" gate="R" pin="2"/>
<pinref part="CON1" gate="CON" pin="5"/>
<wire x1="297.18" y1="203.2" x2="154.94" y2="203.2" width="0.1524" layer="91"/>
<wire x1="154.94" y1="203.2" x2="154.94" y2="162.56" width="0.1524" layer="91"/>
<junction x="154.94" y="162.56"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC3(ADC3)"/>
<wire x1="231.14" y1="154.94" x2="259.08" y2="154.94" width="0.1524" layer="91"/>
<pinref part="JUM2" gate="JUMP" pin="4"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC4(ADC4/SDA)"/>
<pinref part="JUM19" gate="JUMP" pin="1"/>
<wire x1="231.14" y1="152.4" x2="292.1" y2="152.4" width="0.1524" layer="91"/>
<wire x1="292.1" y1="152.4" x2="292.1" y2="149.86" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC5(ADC5/SCL)"/>
<wire x1="231.14" y1="149.86" x2="287.02" y2="149.86" width="0.1524" layer="91"/>
<wire x1="287.02" y1="149.86" x2="287.02" y2="147.32" width="0.1524" layer="91"/>
<pinref part="JUM19" gate="JUMP" pin="2"/>
<wire x1="287.02" y1="147.32" x2="292.1" y2="147.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD0(RXD)"/>
<wire x1="231.14" y1="139.7" x2="259.08" y2="139.7" width="0.1524" layer="91"/>
<pinref part="JUM12" gate="JUM" pin="1"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD1(TXD)"/>
<wire x1="231.14" y1="137.16" x2="259.08" y2="137.16" width="0.1524" layer="91"/>
<pinref part="JUM12" gate="JUM" pin="2"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD2(INT0)"/>
<wire x1="231.14" y1="134.62" x2="259.08" y2="134.62" width="0.1524" layer="91"/>
<pinref part="JUM12" gate="JUM" pin="3"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD3(INT1)"/>
<wire x1="231.14" y1="132.08" x2="259.08" y2="132.08" width="0.1524" layer="91"/>
<pinref part="JUM12" gate="JUM" pin="4"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD4(XCK/T0)"/>
<wire x1="231.14" y1="129.54" x2="259.08" y2="129.54" width="0.1524" layer="91"/>
<pinref part="JUM12" gate="JUM" pin="5"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD5(T1)"/>
<wire x1="231.14" y1="127" x2="259.08" y2="127" width="0.1524" layer="91"/>
<pinref part="JUM12" gate="JUM" pin="6"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD6(AIN0)"/>
<wire x1="231.14" y1="124.46" x2="259.08" y2="124.46" width="0.1524" layer="91"/>
<pinref part="JUM12" gate="JUM" pin="7"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD7(AIN1)"/>
<wire x1="231.14" y1="121.92" x2="274.32" y2="121.92" width="0.1524" layer="91"/>
<pinref part="JUM13" gate="JUMP" pin="1"/>
<junction x="274.32" y="121.92"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB0(ICP)"/>
<wire x1="231.14" y1="116.84" x2="259.08" y2="116.84" width="0.1524" layer="91"/>
<pinref part="JUM11" gate="JUMP" pin="1"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB1(OC1A)"/>
<wire x1="231.14" y1="114.3" x2="259.08" y2="114.3" width="0.1524" layer="91"/>
<pinref part="JUM11" gate="JUMP" pin="2"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB3(MOSI/OC2)"/>
<wire x1="231.14" y1="109.22" x2="246.38" y2="109.22" width="0.1524" layer="91"/>
<pinref part="JUM11" gate="JUMP" pin="4"/>
<wire x1="246.38" y1="109.22" x2="259.08" y2="109.22" width="0.1524" layer="91"/>
<wire x1="246.38" y1="109.22" x2="246.38" y2="86.36" width="0.1524" layer="91"/>
<wire x1="246.38" y1="86.36" x2="317.5" y2="86.36" width="0.1524" layer="91"/>
<wire x1="317.5" y1="86.36" x2="317.5" y2="193.04" width="0.1524" layer="91"/>
<wire x1="317.5" y1="193.04" x2="287.02" y2="193.04" width="0.1524" layer="91"/>
<wire x1="287.02" y1="193.04" x2="287.02" y2="208.28" width="0.1524" layer="91"/>
<pinref part="CON1" gate="CON" pin="1"/>
<wire x1="287.02" y1="208.28" x2="297.18" y2="208.28" width="0.1524" layer="91"/>
<junction x="246.38" y="109.22"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB2(SS/OC1B)"/>
<wire x1="231.14" y1="111.76" x2="259.08" y2="111.76" width="0.1524" layer="91"/>
<pinref part="JUM11" gate="JUMP" pin="3"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB4(MISO)"/>
<wire x1="231.14" y1="106.68" x2="241.3" y2="106.68" width="0.1524" layer="91"/>
<pinref part="JUM11" gate="JUMP" pin="5"/>
<wire x1="241.3" y1="106.68" x2="259.08" y2="106.68" width="0.1524" layer="91"/>
<wire x1="241.3" y1="106.68" x2="241.3" y2="81.28" width="0.1524" layer="91"/>
<wire x1="241.3" y1="81.28" x2="325.12" y2="81.28" width="0.1524" layer="91"/>
<wire x1="325.12" y1="81.28" x2="325.12" y2="215.9" width="0.1524" layer="91"/>
<wire x1="325.12" y1="215.9" x2="279.4" y2="215.9" width="0.1524" layer="91"/>
<wire x1="279.4" y1="215.9" x2="279.4" y2="198.12" width="0.1524" layer="91"/>
<pinref part="CON1" gate="CON" pin="9"/>
<wire x1="279.4" y1="198.12" x2="297.18" y2="198.12" width="0.1524" layer="91"/>
<junction x="241.3" y="106.68"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB5(SCK)"/>
<wire x1="231.14" y1="104.14" x2="236.22" y2="104.14" width="0.1524" layer="91"/>
<pinref part="JUM14" gate="JUMP" pin="2"/>
<junction x="274.32" y="104.14"/>
<wire x1="236.22" y1="104.14" x2="274.32" y2="104.14" width="0.1524" layer="91"/>
<wire x1="236.22" y1="104.14" x2="236.22" y2="78.74" width="0.1524" layer="91"/>
<wire x1="236.22" y1="78.74" x2="327.66" y2="78.74" width="0.1524" layer="91"/>
<wire x1="327.66" y1="78.74" x2="327.66" y2="218.44" width="0.1524" layer="91"/>
<wire x1="327.66" y1="218.44" x2="281.94" y2="218.44" width="0.1524" layer="91"/>
<wire x1="281.94" y1="218.44" x2="281.94" y2="200.66" width="0.1524" layer="91"/>
<pinref part="CON1" gate="CON" pin="7"/>
<wire x1="281.94" y1="200.66" x2="297.18" y2="200.66" width="0.1524" layer="91"/>
<junction x="236.22" y="104.14"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="JUM5" gate="JUMP" pin="1"/>
<wire x1="203.2" y1="63.5" x2="203.2" y2="48.26" width="0.1524" layer="91"/>
<wire x1="203.2" y1="48.26" x2="248.92" y2="48.26" width="0.1524" layer="91"/>
<pinref part="PAD7" gate="PAD" pin="-"/>
<wire x1="248.92" y1="66.04" x2="248.92" y2="48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$29" class="0">
<segment>
<pinref part="JUM4" gate="JUMP" pin="3"/>
<wire x1="43.18" y1="144.78" x2="45.72" y2="144.78" width="0.1524" layer="91"/>
<wire x1="45.72" y1="144.78" x2="45.72" y2="149.86" width="0.1524" layer="91"/>
<wire x1="45.72" y1="149.86" x2="5.08" y2="149.86" width="0.1524" layer="91"/>
<wire x1="5.08" y1="149.86" x2="5.08" y2="20.32" width="0.1524" layer="91"/>
<wire x1="5.08" y1="20.32" x2="208.28" y2="20.32" width="0.1524" layer="91"/>
<wire x1="208.28" y1="20.32" x2="208.28" y2="38.1" width="0.1524" layer="91"/>
<pinref part="JUM20" gate="JUMP" pin="1"/>
<wire x1="208.28" y1="38.1" x2="200.66" y2="38.1" width="0.1524" layer="91"/>
<wire x1="200.66" y1="38.1" x2="200.66" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$30" class="0">
<segment>
<pinref part="R2" gate="R" pin="1"/>
<wire x1="55.88" y1="132.08" x2="55.88" y2="127" width="0.1524" layer="91"/>
<pinref part="D3" gate="D" pin="A"/>
</segment>
</net>
<net name="N$31" class="0">
<segment>
<pinref part="R3" gate="R" pin="1"/>
<pinref part="D2" gate="D" pin="A"/>
<wire x1="134.62" y1="48.26" x2="134.62" y2="53.34" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$33" class="0">
<segment>
<pinref part="JUM5" gate="JUMP" pin="2"/>
<wire x1="205.74" y1="63.5" x2="205.74" y2="50.8" width="0.1524" layer="91"/>
<wire x1="243.84" y1="50.8" x2="243.84" y2="66.04" width="0.1524" layer="91"/>
<wire x1="205.74" y1="50.8" x2="243.84" y2="50.8" width="0.1524" layer="91"/>
<pinref part="PAD3" gate="PAD" pin="+"/>
</segment>
</net>
<net name="N$26" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB6(XTAL1/TOSC1)"/>
<wire x1="172.72" y1="144.78" x2="182.88" y2="144.78" width="0.1524" layer="91"/>
<pinref part="JUM9" gate="JUMP" pin="2"/>
</segment>
</net>
<net name="N$34" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB7(XTAL2/TOSC2)"/>
<wire x1="172.72" y1="139.7" x2="182.88" y2="139.7" width="0.1524" layer="91"/>
<pinref part="JUM9" gate="JUMP" pin="1"/>
<wire x1="172.72" y1="142.24" x2="172.72" y2="139.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$32" class="0">
<segment>
<pinref part="PAD5" gate="PAD" pin="+"/>
<pinref part="C2" gate="C" pin="C_EL+"/>
<wire x1="66.04" y1="68.58" x2="58.42" y2="68.58" width="0.1524" layer="91"/>
<wire x1="58.42" y1="68.58" x2="58.42" y2="50.8" width="0.1524" layer="91"/>
<junction x="58.42" y="68.58"/>
<wire x1="48.26" y1="68.58" x2="53.34" y2="68.58" width="0.1524" layer="91"/>
<pinref part="JUM6" gate="JUMP" pin="2"/>
<wire x1="53.34" y1="68.58" x2="58.42" y2="68.58" width="0.1524" layer="91"/>
<wire x1="48.26" y1="50.8" x2="53.34" y2="50.8" width="0.1524" layer="91"/>
<wire x1="53.34" y1="50.8" x2="53.34" y2="68.58" width="0.1524" layer="91"/>
<junction x="53.34" y="68.58"/>
<wire x1="66.04" y1="68.58" x2="66.04" y2="76.2" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="IN"/>
<wire x1="66.04" y1="76.2" x2="71.12" y2="76.2" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$28" class="0">
<segment>
<pinref part="D4" gate="D" pin="A"/>
<pinref part="R5" gate="R" pin="1"/>
<wire x1="292.1" y1="121.92" x2="289.56" y2="121.92" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$35" class="0">
<segment>
<pinref part="D5" gate="D" pin="A"/>
<pinref part="R4" gate="R" pin="1"/>
<wire x1="289.56" y1="104.14" x2="292.1" y2="104.14" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$37" class="0">
<segment>
<pinref part="R7" gate="R" pin="2"/>
<pinref part="JUM1" gate="JUMP" pin="2"/>
<wire x1="111.76" y1="205.74" x2="111.76" y2="203.2" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$38" class="0">
<segment>
<pinref part="R6" gate="R" pin="2"/>
<pinref part="JUM1" gate="JUMP" pin="3"/>
<wire x1="119.38" y1="203.2" x2="114.3" y2="203.2" width="0.1524" layer="91"/>
<wire x1="114.3" y1="203.2" x2="114.3" y2="205.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$39" class="0">
<segment>
<pinref part="JUM1" gate="JUMP" pin="4"/>
<pinref part="R8" gate="R" pin="2"/>
<wire x1="116.84" y1="205.74" x2="127" y2="205.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC2(ADC2)"/>
<wire x1="231.14" y1="157.48" x2="233.68" y2="157.48" width="0.1524" layer="91"/>
<wire x1="233.68" y1="157.48" x2="259.08" y2="157.48" width="0.1524" layer="91"/>
<junction x="233.68" y="157.48"/>
<wire x1="233.68" y1="177.8" x2="233.68" y2="157.48" width="0.1524" layer="91"/>
<wire x1="170.18" y1="177.8" x2="233.68" y2="177.8" width="0.1524" layer="91"/>
<pinref part="R10" gate="R" pin="1"/>
<pinref part="JUM2" gate="JUMP" pin="3"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC1(ADC1)"/>
<wire x1="231.14" y1="160.02" x2="241.3" y2="160.02" width="0.1524" layer="91"/>
<wire x1="241.3" y1="160.02" x2="259.08" y2="160.02" width="0.1524" layer="91"/>
<junction x="241.3" y="160.02"/>
<wire x1="241.3" y1="182.88" x2="241.3" y2="160.02" width="0.1524" layer="91"/>
<wire x1="185.42" y1="182.88" x2="241.3" y2="182.88" width="0.1524" layer="91"/>
<pinref part="R11" gate="R" pin="1"/>
<pinref part="JUM2" gate="JUMP" pin="2"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC0(ADC0)"/>
<wire x1="231.14" y1="162.56" x2="256.54" y2="162.56" width="0.1524" layer="91"/>
<junction x="256.54" y="162.56"/>
<wire x1="256.54" y1="162.56" x2="259.08" y2="162.56" width="0.1524" layer="91"/>
<wire x1="256.54" y1="187.96" x2="256.54" y2="162.56" width="0.1524" layer="91"/>
<wire x1="198.12" y1="187.96" x2="256.54" y2="187.96" width="0.1524" layer="91"/>
<pinref part="R12" gate="R" pin="1"/>
<pinref part="JUM2" gate="JUMP" pin="1"/>
</segment>
</net>
<net name="N$36" class="0">
<segment>
<pinref part="R5" gate="R" pin="2"/>
<pinref part="JUM13" gate="JUMP" pin="2"/>
<wire x1="276.86" y1="121.92" x2="279.4" y2="121.92" width="0.1524" layer="91"/>
<junction x="276.86" y="121.92"/>
</segment>
</net>
<net name="N$40" class="0">
<segment>
<pinref part="R4" gate="R" pin="2"/>
<pinref part="JUM14" gate="JUMP" pin="1"/>
<wire x1="276.86" y1="104.14" x2="279.4" y2="104.14" width="0.1524" layer="91"/>
<junction x="276.86" y="104.14"/>
</segment>
</net>
<net name="N$41" class="0">
<segment>
<pinref part="JUM15" gate="JUMP" pin="1"/>
<wire x1="154.94" y1="48.26" x2="154.94" y2="38.1" width="0.1524" layer="91"/>
<wire x1="154.94" y1="38.1" x2="190.5" y2="38.1" width="0.1524" layer="91"/>
<wire x1="190.5" y1="38.1" x2="190.5" y2="35.56" width="0.1524" layer="91"/>
<pinref part="JUM20" gate="JUMP" pin="5"/>
</segment>
</net>
<net name="N$42" class="0">
<segment>
<pinref part="JUM15" gate="JUMP" pin="2"/>
<wire x1="157.48" y1="48.26" x2="157.48" y2="40.64" width="0.1524" layer="91"/>
<wire x1="157.48" y1="40.64" x2="193.04" y2="40.64" width="0.1524" layer="91"/>
<wire x1="193.04" y1="40.64" x2="193.04" y2="35.56" width="0.1524" layer="91"/>
<pinref part="JUM20" gate="JUMP" pin="4"/>
</segment>
</net>
<net name="N$43" class="0">
<segment>
<pinref part="JUM15" gate="JUMP" pin="3"/>
<wire x1="160.02" y1="48.26" x2="160.02" y2="43.18" width="0.1524" layer="91"/>
<wire x1="160.02" y1="43.18" x2="195.58" y2="43.18" width="0.1524" layer="91"/>
<wire x1="195.58" y1="43.18" x2="195.58" y2="35.56" width="0.1524" layer="91"/>
<pinref part="JUM20" gate="JUMP" pin="3"/>
</segment>
</net>
<net name="N$44" class="0">
<segment>
<wire x1="198.12" y1="45.72" x2="198.12" y2="35.56" width="0.1524" layer="91"/>
<pinref part="JUM15" gate="JUMP" pin="4"/>
<wire x1="162.56" y1="48.26" x2="162.56" y2="45.72" width="0.1524" layer="91"/>
<wire x1="162.56" y1="45.72" x2="198.12" y2="45.72" width="0.1524" layer="91"/>
<pinref part="JUM20" gate="JUMP" pin="2"/>
</segment>
</net>
<net name="N$48" class="0">
<segment>
<pinref part="IO1" gate="IO" pin="OUT"/>
<pinref part="C4" gate="C" pin="C_EL+"/>
<wire x1="93.98" y1="142.24" x2="104.14" y2="142.24" width="0.1524" layer="91"/>
<wire x1="104.14" y1="142.24" x2="104.14" y2="134.62" width="0.1524" layer="91"/>
<wire x1="104.14" y1="142.24" x2="109.22" y2="142.24" width="0.1524" layer="91"/>
<junction x="104.14" y="142.24"/>
<wire x1="109.22" y1="142.24" x2="111.76" y2="142.24" width="0.1524" layer="91"/>
<wire x1="111.76" y1="142.24" x2="114.3" y2="142.24" width="0.1524" layer="91"/>
<wire x1="114.3" y1="142.24" x2="116.84" y2="142.24" width="0.1524" layer="91"/>
<wire x1="116.84" y1="142.24" x2="119.38" y2="142.24" width="0.1524" layer="91"/>
<junction x="109.22" y="142.24"/>
<junction x="111.76" y="142.24"/>
<junction x="114.3" y="142.24"/>
<junction x="116.84" y="142.24"/>
<pinref part="R7" gate="R" pin="1"/>
<pinref part="R6" gate="R" pin="1"/>
<wire x1="111.76" y1="193.04" x2="111.76" y2="177.8" width="0.1524" layer="91"/>
<wire x1="111.76" y1="177.8" x2="111.76" y2="142.24" width="0.1524" layer="91"/>
<wire x1="119.38" y1="193.04" x2="114.3" y2="193.04" width="0.1524" layer="91"/>
<wire x1="114.3" y1="193.04" x2="114.3" y2="182.88" width="0.1524" layer="91"/>
<pinref part="R8" gate="R" pin="1"/>
<wire x1="114.3" y1="182.88" x2="114.3" y2="142.24" width="0.1524" layer="91"/>
<wire x1="127" y1="195.58" x2="127" y2="190.5" width="0.1524" layer="91"/>
<wire x1="127" y1="190.5" x2="116.84" y2="190.5" width="0.1524" layer="91"/>
<wire x1="116.84" y1="190.5" x2="116.84" y2="187.96" width="0.1524" layer="91"/>
<wire x1="116.84" y1="187.96" x2="116.84" y2="142.24" width="0.1524" layer="91"/>
<wire x1="109.22" y1="142.24" x2="109.22" y2="205.74" width="0.1524" layer="91"/>
<pinref part="JUM1" gate="JUMP" pin="1"/>
<wire x1="111.76" y1="177.8" x2="160.02" y2="177.8" width="0.1524" layer="91"/>
<junction x="111.76" y="177.8"/>
<junction x="114.3" y="182.88"/>
<junction x="116.84" y="187.96"/>
<wire x1="114.3" y1="182.88" x2="175.26" y2="182.88" width="0.1524" layer="91"/>
<wire x1="116.84" y1="187.96" x2="187.96" y2="187.96" width="0.1524" layer="91"/>
<pinref part="R10" gate="R" pin="2"/>
<pinref part="R11" gate="R" pin="2"/>
<pinref part="R12" gate="R" pin="2"/>
<pinref part="IC1" gate="G$1" pin="VCC@1"/>
<wire x1="182.88" y1="127" x2="182.88" y2="114.3" width="0.1524" layer="91"/>
<wire x1="182.88" y1="114.3" x2="162.56" y2="114.3" width="0.1524" layer="91"/>
<pinref part="C7" gate="C" pin="1"/>
<wire x1="162.56" y1="114.3" x2="160.02" y2="114.3" width="0.1524" layer="91"/>
<wire x1="160.02" y1="114.3" x2="149.86" y2="114.3" width="0.1524" layer="91"/>
<wire x1="149.86" y1="114.3" x2="129.54" y2="114.3" width="0.1524" layer="91"/>
<wire x1="129.54" y1="114.3" x2="119.38" y2="114.3" width="0.1524" layer="91"/>
<wire x1="149.86" y1="114.3" x2="149.86" y2="119.38" width="0.1524" layer="91"/>
<junction x="149.86" y="114.3"/>
<junction x="160.02" y="114.3"/>
<pinref part="R1" gate="R" pin="1"/>
<wire x1="129.54" y1="121.92" x2="129.54" y2="114.3" width="0.1524" layer="91"/>
<junction x="129.54" y="114.3"/>
<wire x1="119.38" y1="142.24" x2="119.38" y2="114.3" width="0.1524" layer="91"/>
<wire x1="162.56" y1="114.3" x2="162.56" y2="88.9" width="0.1524" layer="91"/>
<wire x1="162.56" y1="88.9" x2="233.68" y2="88.9" width="0.1524" layer="91"/>
<wire x1="233.68" y1="88.9" x2="233.68" y2="76.2" width="0.1524" layer="91"/>
<wire x1="233.68" y1="76.2" x2="330.2" y2="76.2" width="0.1524" layer="91"/>
<wire x1="330.2" y1="76.2" x2="330.2" y2="208.28" width="0.1524" layer="91"/>
<pinref part="CON1" gate="CON" pin="2"/>
<wire x1="330.2" y1="208.28" x2="312.42" y2="208.28" width="0.1524" layer="91"/>
<junction x="162.56" y="114.3"/>
<pinref part="R9" gate="R" pin="1"/>
<wire x1="160.02" y1="119.38" x2="160.02" y2="114.3" width="0.1524" layer="91"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="IO1" gate="IO" pin="IN"/>
<pinref part="C3" gate="C" pin="C_EL+"/>
<wire x1="53.34" y1="142.24" x2="55.88" y2="142.24" width="0.1524" layer="91"/>
<wire x1="55.88" y1="142.24" x2="63.5" y2="142.24" width="0.1524" layer="91"/>
<wire x1="63.5" y1="142.24" x2="73.66" y2="142.24" width="0.1524" layer="91"/>
<wire x1="63.5" y1="134.62" x2="63.5" y2="142.24" width="0.1524" layer="91"/>
<junction x="63.5" y="142.24"/>
<wire x1="35.56" y1="88.9" x2="53.34" y2="88.9" width="0.1524" layer="91"/>
<wire x1="53.34" y1="88.9" x2="53.34" y2="142.24" width="0.1524" layer="91"/>
<junction x="53.34" y="142.24"/>
<pinref part="JUM4" gate="JUMP" pin="2"/>
<wire x1="53.34" y1="142.24" x2="43.18" y2="142.24" width="0.1524" layer="91"/>
<pinref part="R2" gate="R" pin="2"/>
<junction x="55.88" y="142.24"/>
<pinref part="CONN1" gate="G$1" pin="VCC"/>
<junction x="35.56" y="88.9"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
