package org.firstinspires.ftc.teamcode.specialized;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Carousel Duck Park")
public class AutoOpV4_BlueCarouselDuckPark extends AutoOpV4_RedCarouselDuckPark{
	@Override
	protected void setParams() {
		side = Side.BLUE;
	}
}
