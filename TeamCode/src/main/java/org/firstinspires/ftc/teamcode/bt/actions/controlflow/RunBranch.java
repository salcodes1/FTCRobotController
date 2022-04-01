package org.firstinspires.ftc.teamcode.bt.actions.controlflow;

import org.firstinspires.ftc.teamcode.bt.Action;
import org.firstinspires.ftc.teamcode.bt.AutonomousOpMode;

public class RunBranch extends Action {


	private Action cond1;
	private Action cond2;
	private Action outcome1;
	private Action outcome2;

	private boolean oneFinished;
	private Action outcome;

	public RunBranch(Action cond1, Action cond2, Action outcome1, Action outcome2) {
		this.cond1 = cond1;
		this.cond2 = cond2;
		this.outcome1 = outcome1;
		this.outcome2 = outcome2;
	}

	@Override
	protected void _start(AutonomousOpMode context) {
		cond1.start(context);
		cond2.start(context);
	}

	@Override
	protected void _tick(AutonomousOpMode context) {
		if(!oneFinished) {
			if(cond1.hasFinished(context)) {
				oneFinished = true;
				outcome = outcome1;
				outcome.start(context);


				cond1.end(context);
				cond2.end(context);
			} else if(cond2.hasFinished(context)) {
				oneFinished = true;
				outcome = outcome2;
				outcome.start(context);


				cond1.end(context);
				cond2.end(context);
			}
			else if(!cond1.hasFinished(context) && !cond2.hasFinished(context))
			{
				cond1.tick(context);
				cond2.tick(context);
			}

		}

	}

	@Override
	protected boolean _hasFinished(AutonomousOpMode context) {
		return outcome != null && outcome.hasFinished(context);
	}

	@Override
	protected void _end(AutonomousOpMode context) {
		if(oneFinished)
			outcome.end(context);
		else {
			cond1.end(context);
			cond2.end(context);
		}
	}
}
