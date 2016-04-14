<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.5.0">
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
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="wirepad">
<description>&lt;b&gt;Single Pads&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="2,54/0,9">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<wire x1="-1.27" y1="1.27" x2="-0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.762" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="0.762" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-0.762" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="0.762" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.762" y1="-1.27" x2="-1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.762" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.635" width="0.1524" layer="51"/>
<pad name="1" x="0" y="0" drill="0.9144" diameter="2.54" shape="octagon"/>
<text x="-1.27" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.6" size="0.0254" layer="27">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="PAD">
<wire x1="-1.016" y1="1.016" x2="1.016" y2="-1.016" width="0.254" layer="94"/>
<wire x1="-1.016" y1="-1.016" x2="1.016" y2="1.016" width="0.254" layer="94"/>
<text x="-1.143" y="1.8542" size="1.778" layer="95">&gt;NAME</text>
<text x="-1.143" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<pin name="P" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="2,54/0,9" prefix="PAD" uservalue="yes">
<description>&lt;b&gt;THROUGH-HOLE PAD&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="PAD" x="0" y="0"/>
</gates>
<devices>
<device name="" package="2,54/0,9">
<connects>
<connect gate="1" pin="P" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-lstb">
<description>&lt;b&gt;Pin Headers&lt;/b&gt;&lt;p&gt;
Naming:&lt;p&gt;
MA = male&lt;p&gt;
# contacts - # rows&lt;p&gt;
W = angled&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="MA03-2">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-3.175" y1="2.54" x2="-1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="2.54" x2="-1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.905" x2="-0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.54" x2="0.635" y2="2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.54" x2="1.27" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="2.54" x2="-3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.905" x2="1.905" y2="2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="2.54" x2="3.175" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="2.54" x2="3.81" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.54" x2="-0.635" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.54" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.905" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-2.54" x2="-3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.175" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-2.54" x2="1.905" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-2.54" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="-1.27" drill="1.016"/>
<pad name="3" x="0" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="5" x="2.54" y="-1.27" drill="1.016" shape="octagon"/>
<pad name="2" x="-2.54" y="1.27" drill="1.016" shape="octagon"/>
<pad name="4" x="0" y="1.27" drill="1.016" shape="octagon"/>
<pad name="6" x="2.54" y="1.27" drill="1.016" shape="octagon"/>
<text x="-3.175" y="-4.191" size="1.27" layer="21" ratio="10">1</text>
<text x="-3.81" y="2.921" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="4.064" y="0.635" size="1.27" layer="21" ratio="10">6</text>
<text x="-1.27" y="-4.191" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-1.524" x2="0.254" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="-1.524" x2="-2.286" y2="-1.016" layer="51"/>
<rectangle x1="2.286" y1="-1.524" x2="2.794" y2="-1.016" layer="51"/>
<rectangle x1="-2.794" y1="1.016" x2="-2.286" y2="1.524" layer="51"/>
<rectangle x1="-0.254" y1="1.016" x2="0.254" y2="1.524" layer="51"/>
<rectangle x1="2.286" y1="1.016" x2="2.794" y2="1.524" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="MA03-2">
<wire x1="3.81" y1="-5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="-3.81" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-3.81" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="-1.27" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="0" x2="-1.27" y2="0" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="-2.54" x2="-1.27" y2="-2.54" width="0.6096" layer="94"/>
<text x="-3.81" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="-3.81" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="-7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="4" x="-7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1"/>
<pin name="6" x="-7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MA03-2" prefix="SV" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="MA03-2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA03-2">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="1" pin="3" pad="3"/>
<connect gate="1" pin="4" pad="4"/>
<connect gate="1" pin="5" pad="5"/>
<connect gate="1" pin="6" pad="6"/>
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
<library name="supply2">
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
<symbol name="GND">
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0" y1="-1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<text x="-1.905" y="-3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" prefix="SUPPLY">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="GND" symbol="GND" x="0" y="0"/>
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
<library name="solpad">
<description>&lt;b&gt;Solder Pads/Test Points&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="SE14">
<description>&lt;b&gt;SOLDER PAD&lt;/b&gt;&lt;p&gt;
drill 1.4 mm</description>
<wire x1="-1.524" y1="0.635" x2="-1.524" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.524" x2="1.524" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.524" x2="1.524" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.524" y1="0.635" x2="1.524" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="0.635" x2="-0.635" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.524" x2="0.635" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.524" x2="-1.524" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.524" x2="-0.635" y2="-1.524" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.1524" layer="51"/>
<circle x="0" y="0" radius="0.381" width="0.254" layer="51"/>
<pad name="MP" x="0" y="0" drill="1.397" diameter="2.54" shape="octagon"/>
<text x="-1.397" y="1.778" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.381" size="0.0254" layer="27">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="LSP">
<wire x1="-1.016" y1="2.032" x2="1.016" y2="0" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0" x2="1.016" y2="2.032" width="0.254" layer="94"/>
<circle x="0" y="1.016" radius="1.016" width="0.4064" layer="94"/>
<text x="-1.27" y="2.921" size="1.778" layer="95">&gt;NAME</text>
<pin name="MP" x="0" y="-2.54" visible="off" length="short" direction="pas" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SE14" prefix="LSP">
<description>&lt;b&gt;SOLDER PAD&lt;/b&gt;&lt;p&gt; E1553,  drill 1,4mm, distributor Buerklin, 07F820</description>
<gates>
<gate name="1" symbol="LSP" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SE14">
<connects>
<connect gate="1" pin="MP" pad="MP"/>
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
<part name="LEFT_ORANGE_STRIPE" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="RIGHT_YELLOW_STRIPE" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="RIGHT_ORANGE_STRIPE" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="BLACK_TOP" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="VCC" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="SV1" library="con-lstb" deviceset="MA03-2" device=""/>
<part name="SV2" library="con-lstb" deviceset="MA03-2" device=""/>
<part name="LEFT_YELLOW_STRIPE" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="PAD7" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="PAD8" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="YELLOW" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="GREEN" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="EN1B" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="EN1A" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="EN2B" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="EN2A" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="GND" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="MIDDLE_BLACK" library="wirepad" deviceset="2,54/0,9" device=""/>
<part name="SUPPLY6" library="supply2" deviceset="GND" device=""/>
<part name="LSP1" library="solpad" deviceset="SE14" device=""/>
<part name="LSP2" library="solpad" deviceset="SE14" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="LEFT_ORANGE_STRIPE" gate="1" x="109.22" y="27.94"/>
<instance part="RIGHT_YELLOW_STRIPE" gate="1" x="109.22" y="20.32"/>
<instance part="RIGHT_ORANGE_STRIPE" gate="1" x="109.22" y="12.7"/>
<instance part="BLACK_TOP" gate="1" x="172.72" y="15.24"/>
<instance part="VCC" gate="1" x="-63.754" y="4.064"/>
<instance part="SV1" gate="1" x="2.54" y="33.02" rot="R270"/>
<instance part="SV2" gate="1" x="60.96" y="33.02" rot="R270"/>
<instance part="LEFT_YELLOW_STRIPE" gate="1" x="109.22" y="35.56"/>
<instance part="PAD7" gate="1" x="172.72" y="30.48"/>
<instance part="PAD8" gate="1" x="172.72" y="38.1"/>
<instance part="YELLOW" gate="1" x="-63.5" y="48.26"/>
<instance part="GREEN" gate="1" x="-63.5" y="40.64"/>
<instance part="EN1B" gate="1" x="-63.5" y="33.02"/>
<instance part="EN1A" gate="1" x="-63.5" y="25.4"/>
<instance part="EN2B" gate="1" x="-63.5" y="17.78"/>
<instance part="EN2A" gate="1" x="-63.5" y="10.16"/>
<instance part="GND" gate="1" x="-63.5" y="-2.54"/>
<instance part="MIDDLE_BLACK" gate="1" x="109.22" y="-2.54"/>
<instance part="SUPPLY6" gate="GND" x="-53.34" y="-5.08"/>
<instance part="LSP1" gate="1" x="-7.62" y="76.2"/>
<instance part="LSP2" gate="1" x="2.54" y="76.2"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="GND" gate="1" pin="P"/>
<wire x1="-60.96" y1="-2.54" x2="-53.34" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="SUPPLY6" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="SV1" gate="1" pin="3"/>
<wire x1="2.54" y1="25.4" x2="2.54" y2="17.78" width="0.1524" layer="91"/>
<label x="0" y="15.24" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="SV2" gate="1" pin="3"/>
<wire x1="60.96" y1="25.4" x2="60.96" y2="17.78" width="0.1524" layer="91"/>
<label x="58.42" y="15.24" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="MIDDLE_BLACK" gate="1" pin="P"/>
<wire x1="111.76" y1="-2.54" x2="139.7" y2="-2.54" width="0.1524" layer="91"/>
<label x="139.7" y="-2.54" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="BLACK_TOP" gate="1" pin="P"/>
<wire x1="175.26" y1="15.24" x2="187.96" y2="15.24" width="0.1524" layer="91"/>
<label x="187.96" y="15.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="EN2B" class="0">
<segment>
<pinref part="SV2" gate="1" pin="5"/>
<wire x1="63.5" y1="25.4" x2="71.12" y2="17.78" width="0.1524" layer="91"/>
<label x="71.12" y="17.78" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="EN2B" gate="1" pin="P"/>
<wire x1="-60.96" y1="17.78" x2="-53.34" y2="17.78" width="0.1524" layer="91"/>
<label x="-53.34" y="17.78" size="1.778" layer="95"/>
</segment>
</net>
<net name="EN2A" class="0">
<segment>
<pinref part="SV2" gate="1" pin="6"/>
<wire x1="63.5" y1="40.64" x2="68.58" y2="48.26" width="0.1524" layer="91"/>
<label x="76.2" y="48.26" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="EN2A" gate="1" pin="P"/>
<wire x1="-60.96" y1="10.16" x2="-53.34" y2="10.16" width="0.1524" layer="91"/>
<label x="-53.34" y="10.16" size="1.778" layer="95"/>
</segment>
</net>
<net name="EN1B" class="0">
<segment>
<pinref part="SV1" gate="1" pin="5"/>
<wire x1="5.08" y1="25.4" x2="12.7" y2="17.78" width="0.1524" layer="91"/>
<label x="20.32" y="17.78" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="EN1B" gate="1" pin="P"/>
<wire x1="-60.96" y1="33.02" x2="-53.34" y2="33.02" width="0.1524" layer="91"/>
<label x="-53.34" y="33.02" size="1.778" layer="95"/>
</segment>
</net>
<net name="EN1A" class="0">
<segment>
<pinref part="SV1" gate="1" pin="6"/>
<wire x1="5.08" y1="40.64" x2="12.7" y2="40.64" width="0.1524" layer="91"/>
<label x="17.78" y="43.18" size="1.778" layer="95" rot="R180"/>
</segment>
<segment>
<pinref part="EN1A" gate="1" pin="P"/>
<wire x1="-60.96" y1="25.4" x2="-53.34" y2="25.4" width="0.1524" layer="91"/>
<label x="-53.34" y="25.4" size="1.778" layer="95"/>
</segment>
</net>
<net name="MOTOR1-" class="0">
<segment>
<pinref part="SV1" gate="1" pin="1"/>
<wire x1="0" y1="25.4" x2="-5.08" y2="20.32" width="0.1524" layer="91"/>
<label x="-15.24" y="17.78" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="LEFT_YELLOW_STRIPE" gate="1" pin="P"/>
<wire x1="111.76" y1="35.56" x2="139.7" y2="35.56" width="0.1524" layer="91"/>
<label x="139.7" y="35.56" size="1.778" layer="95"/>
</segment>
</net>
<net name="MOTOR1+" class="0">
<segment>
<pinref part="SV1" gate="1" pin="2"/>
<wire x1="0" y1="40.64" x2="-7.62" y2="40.64" width="0.1524" layer="91"/>
<label x="-15.24" y="40.64" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="LEFT_ORANGE_STRIPE" gate="1" pin="P"/>
<wire x1="111.76" y1="27.94" x2="139.7" y2="27.94" width="0.1524" layer="91"/>
<label x="139.7" y="27.94" size="1.778" layer="95"/>
</segment>
</net>
<net name="MOTOR2-" class="0">
<segment>
<pinref part="SV2" gate="1" pin="1"/>
<wire x1="58.42" y1="25.4" x2="53.34" y2="20.32" width="0.1524" layer="91"/>
<label x="43.18" y="17.78" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="RIGHT_YELLOW_STRIPE" gate="1" pin="P"/>
<wire x1="111.76" y1="20.32" x2="139.7" y2="20.32" width="0.1524" layer="91"/>
<label x="139.7" y="20.32" size="1.778" layer="95"/>
</segment>
</net>
<net name="MOTOR2+" class="0">
<segment>
<pinref part="SV2" gate="1" pin="2"/>
<wire x1="58.42" y1="40.64" x2="53.34" y2="45.72" width="0.1524" layer="91"/>
<label x="38.1" y="45.72" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="RIGHT_ORANGE_STRIPE" gate="1" pin="P"/>
<wire x1="111.76" y1="12.7" x2="139.7" y2="12.7" width="0.1524" layer="91"/>
<label x="139.7" y="12.7" size="1.778" layer="95"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="SV2" gate="1" pin="4"/>
<wire x1="60.96" y1="40.64" x2="60.96" y2="53.34" width="0.1524" layer="91"/>
<label x="58.42" y="53.34" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="VCC" gate="1" pin="P"/>
<wire x1="-61.214" y1="4.064" x2="-52.832" y2="4.064" width="0.1524" layer="91"/>
<label x="-52.07" y="4.064" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="SV1" gate="1" pin="4"/>
<wire x1="2.54" y1="40.64" x2="2.54" y2="48.26" width="0.1524" layer="91"/>
<label x="0" y="48.26" size="1.778" layer="95"/>
</segment>
</net>
<net name="RX" class="0">
<segment>
<pinref part="YELLOW" gate="1" pin="P"/>
<wire x1="-60.96" y1="48.26" x2="-53.34" y2="48.26" width="0.1524" layer="91"/>
<label x="-53.34" y="48.26" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="PAD8" gate="1" pin="P"/>
<wire x1="175.26" y1="38.1" x2="187.96" y2="38.1" width="0.1524" layer="91"/>
<label x="187.96" y="38.1" size="1.778" layer="95"/>
</segment>
</net>
<net name="TX" class="0">
<segment>
<pinref part="PAD7" gate="1" pin="P"/>
<wire x1="175.26" y1="30.48" x2="190.5" y2="30.48" width="0.1524" layer="91"/>
<label x="187.96" y="30.48" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="GREEN" gate="1" pin="P"/>
<wire x1="-60.96" y1="40.64" x2="-53.34" y2="40.64" width="0.1524" layer="91"/>
<label x="-53.34" y="40.64" size="1.778" layer="95"/>
</segment>
</net>
<net name="VCC12" class="0">
<segment>
<pinref part="LSP1" gate="1" pin="MP"/>
<wire x1="-7.62" y1="73.66" x2="-7.62" y2="68.58" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="LSP2" gate="1" pin="MP"/>
<wire x1="2.54" y1="73.66" x2="2.54" y2="68.58" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
