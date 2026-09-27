#include <multi_uav_formation/team_planning_context.h>

#include <array>
#include <iostream>
#include <vector>

int main()
{
  using namespace multi_uav_formation;
  bool ok=true;
  std::array<TeamPlanningMemberContext,3> members;
  for(int drone=0;drone<3;++drone)
  {
    members[drone].drone_id=drone;
    members[drone].planning_generation=10+drone;
    members[drone].expected_execution_generation=20+drone;
    members[drone].expected_trajectory_id=30+drone;
    members[drone].bundle_stamp=100.0+0.01*drone;
    members[drone].target_prediction_epoch=100.30;
    members[drone].dynamic_prediction_epoch=100.30;
    members[drone].target_snapshot_identity=71;
    members[drone].target_snapshot_epoch=100.0+0.01*drone;
    members[drone].dynamic_prediction_identity=81;
    members[drone].dynamic_prediction_valid_from=99.9;
    members[drone].dynamic_prediction_valid_to=102.0;
    members[drone].static_map_revision=7;
    members[drone].visibility_model_version=9;
    members[drone].earliest_activation=100.30;
    members[drone].validated_end=102.0;
  }
  const auto context=makeTeamPlanningContext(
      members,55,100.08,100.30,1.2,100.25,0.10,0.30);
  std::array<std::uint64_t,3> planning{{10,11,12}};
  std::array<std::uint64_t,3> execution{{20,21,22}};
  std::array<int,3> trajectory_ids{{30,31,32}};
  const bool current=teamPlanningContextStillCurrent(
      context,planning,execution,trajectory_ids,100.12);
  auto stale_execution=execution;
  ++stale_execution[1];
  const bool stale_rejected=!teamPlanningContextStillCurrent(
      context,planning,stale_execution,trajectory_ids,100.12);
  auto mismatched_members=members;
  ++mismatched_members[2].static_map_revision;
  const auto mismatched=makeTeamPlanningContext(
      mismatched_members,56,100.08,100.30,1.2,100.25,0.10,0.30);
  auto activation_mismatch_members=members;
  activation_mismatch_members[2].earliest_activation+=0.01;
  const auto activation_mismatch=makeTeamPlanningContext(
      activation_mismatch_members,57,100.08,100.30,1.2,100.25,0.10,0.30);
  ok=context.valid && current && stale_rejected && !mismatched.valid &&
      !activation_mismatch.valid && ok;
  std::cout<<"[early-joint-context] valid="<<context.valid
      <<" current="<<current<<" stale_rejected="<<stale_rejected
      <<" model_mismatch_rejected="<<!mismatched.valid
      <<" activation_mismatch_rejected="<<!activation_mismatch.valid<<'\n';

  std::array<std::vector<TeamTopologySeedDescriptor>,3> seeds;
  for(int drone=0;drone<3;++drone)
    for(int kind=0;kind<3;++kind)
    {
      TeamTopologySeedDescriptor seed;
      seed.drone_id=drone;
      seed.candidate_id=100*drone+kind;
      seed.hypothesis_id=4;
      seed.hypothesis_generation=77;
      seed.kind=kind;
      seed.constructible=true;
      seed.predicted_camera_time=1.0+0.1*kind;
      seed.dynamic_clearance=1.5-0.1*kind;
      seed.transition_burden=kind==0?0.0:1.0;
      seeds[drone].push_back(seed);
    }
  const auto tuples=enumerateTeamTopologyTuples(seeds,4,77);
  const bool all_27=tuples.size()==27;
  const bool deterministic_best=all_27 &&
      tuples.front().candidate_ids==std::array<int,3>{{2,102,202}};
  ok=all_27 && deterministic_best && ok;
  std::cout<<"[early-joint-tuples] count="<<tuples.size()
      <<" deterministic_best="<<deterministic_best<<'\n';

  const bool sacrifice=teamBenefitRequiresIndividualSacrifice(
      {{1.0,1.0,1.0}},{{0.8,1.0,1.0}},2.5,2.8,1.0e-6);
  const bool no_fake_sacrifice=!teamBenefitRequiresIndividualSacrifice(
      {{1.0,1.0,1.0}},{{0.8,1.0,1.0}},2.5,2.5+1.0e-8,1.0e-6);
  ok=sacrifice && no_fake_sacrifice && ok;
  std::cout<<"[early-joint-individual-sacrifice] team_gain="<<sacrifice
      <<" numerical_noise_rejected="<<no_fake_sacrifice<<'\n';

  std::cout<<(ok?"EARLY_JOINT_PRIMARY_CONTRACT_PASS\n"
               :"EARLY_JOINT_PRIMARY_CONTRACT_FAIL\n");
  return ok?0:1;
}
